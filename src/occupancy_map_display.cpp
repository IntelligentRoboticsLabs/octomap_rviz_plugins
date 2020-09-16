/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Julius Kammerl (jkammerl@willowgarage.com)
 *
 */


#include "octomap_rviz_plugin/occupancy_map_display.hpp"

using namespace rviz_common;

namespace octomap_rviz_plugin
{

static const std::size_t max_octree_depth_ = sizeof(unsigned short) * 8;

OccupancyMapDisplay::OccupancyMapDisplay(): 
  rviz_default_plugins::displays::MapDisplay(),
  octree_depth_ (max_octree_depth_)
{

  topic_property_->setName("Octomap Binary Topic");
  QString message_type = QString::fromStdString("");
  //topic_property_->setMessageType(QString::fromStdString(ros::message_traits::datatype<octomap_msgs::msg::Octomap>()));
  topic_property_->setMessageType(message_type);
  topic_property_->setDescription("octomap_msgs::OctomapBinary topic to subscribe to.");

  tree_depth_property_ = new rviz_common::properties::IntProperty("Max. Octree Depth",
                                         octree_depth_,
                                         "Defines the maximum tree depth",
                                         this,
                                         SLOT (updateTreeDepth() ));
  node_ = rclcpp::Node::make_shared("occupancy_map_display_node");
}

OccupancyMapDisplay::~OccupancyMapDisplay()
{
  unsubscribe();
}

void OccupancyMapDisplay::onInitialize()
{
  rviz_default_plugins::displays::MapDisplay::onInitialize();
}

void OccupancyMapDisplay::updateTreeDepth()
{
  octree_depth_ = tree_depth_property_->getInt();
}

void OccupancyMapDisplay::updateTopic()
{
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
}

void OccupancyMapDisplay::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  try
  {
    unsubscribe();

    const std::string& topicStr = topic_property_->getStdString();

    if (!topicStr.empty())
    {
      sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
        topicStr, rclcpp::SystemDefaultsQoS(),
        std::bind(&OccupancyMapDisplay::handleOctomapBinaryMessage, this, std::placeholders::_1));
    }
  }
  catch (std::exception & e)
  {
    setStatus(properties::StatusProperty::Error, "Topic", (std::string("Error subscribing: ") + e.what()).c_str());
  }
}

void OccupancyMapDisplay::unsubscribe()
{
  clear();

  try
  {
    // reset filters
    sub_.reset();
  }
  catch (std::exception & e)
  {
    setStatus(properties::StatusProperty::Error, "Topic", (std::string("Error unsubscribing: ") + e.what()).c_str());
  }
}


template <typename OcTreeType>
void TemplatedOccupancyMapDisplay<OcTreeType>::handleOctomapBinaryMessage(const octomap_msgs::msg::Octomap::SharedPtr msg)
{

  RCLCPP_DEBUG(node_->get_logger(),
   "Received OctomapBinary message (size: %d bytes)", (int)msg->data.size());

  // creating octree
  OcTreeType* octomap = NULL;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
  if (tree){
    octomap = dynamic_cast<OcTreeType*>(tree);
  }

  if (!octomap)
  {
    this->setStatusStd(properties::StatusProperty::Error, "Message", "Failed to create octree structure");
    return;
  }

  // get dimensions of octree
  double minX, minY, minZ, maxX, maxY, maxZ;
  octomap->getMetricMin(minX, minY, minZ);
  octomap->getMetricMax(maxX, maxY, maxZ);
  octomap::point3d minPt = octomap::point3d(minX, minY, minZ);

  unsigned int tree_depth = octomap->getTreeDepth();

  octomap::OcTreeKey paddedMinKey = octomap->coordToKey(minPt);  
  auto occupancy_map = std::make_shared<map_msgs::msg::OccupancyGridUpdate>();

  unsigned int width, height;
  double res = octomap->getNodeSize(octree_depth_);

  unsigned int ds_shift = tree_depth-octree_depth_;

  occupancy_map->header = msg->header;
  occupancy_map->width = width = (maxX-minX) / res + 1;
  occupancy_map->height = height = (maxY-minY) / res + 1;
  occupancy_map->x = minX  - (res / (float)(1<<ds_shift) ) + res;
  occupancy_map->y = minY  - (res / (float)(1<<ds_shift) );

  occupancy_map->data.clear();
  occupancy_map->data.resize(width*height, -1);

    // traverse all leafs in the tree:
  unsigned int treeDepth = std::min<unsigned int>(octree_depth_, octomap->getTreeDepth());
  for (typename OcTreeType::iterator it = octomap->begin(treeDepth), end = octomap->end(); it != end; ++it)
  {
    bool occupied = octomap->isNodeOccupied(*it);
    int intSize = 1 << (octree_depth_ - it.getDepth());

    octomap::OcTreeKey minKey=it.getIndexKey();

    for (int dx = 0; dx < intSize; dx++)
    {
      for (int dy = 0; dy < intSize; dy++)
      {
        int posX = std::max<int>(0, minKey[0] + dx - paddedMinKey[0]);
        posX>>=ds_shift;

        int posY = std::max<int>(0, minKey[1] + dy - paddedMinKey[1]);
        posY>>=ds_shift;

        int idx = width * posY + posX;

        if (occupied)
          occupancy_map->data[idx] = 100;
        else if (occupancy_map->data[idx] == -1)
        {
          occupancy_map->data[idx] = 0;
        }

      }
    }

  }

  delete octomap;

  this->incomingUpdate(occupancy_map);
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(octomap_rviz_plugin::OcTreeMapDisplay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(octomap_rviz_plugin::OcTreeStampedMapDisplay, rviz_common::Display)
