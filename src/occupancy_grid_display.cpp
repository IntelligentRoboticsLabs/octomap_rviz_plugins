/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2020, Intelligent Robotics Lab URJC.
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

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreSharedPtr.h>

#include "rclcpp/time.hpp"

#include "rviz_rendering/material_manager.hpp"
#include "rviz_rendering/objects/grid.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_default_plugins/displays/map/palette_builder.hpp"

#include "octomap_rviz_plugin/occupancy_grid_display.hpp"


namespace octomap_rviz_plugin
{

static const std::size_t max_octree_depth_ = sizeof(unsigned short) * 8;

enum OctreeVoxelRenderMode
{
  OCTOMAP_FREE_VOXELS = 1,
  OCTOMAP_OCCUPIED_VOXELS = 2
};

enum OctreeVoxelColorMode
{
  OCTOMAP_CELL_COLOR,
  OCTOMAP_Z_AXIS_COLOR,
  OCTOMAP_PROBABLILTY_COLOR,
};

OccupancyGridDisplay::OccupancyGridDisplay()
: loaded_(false),
  update_profile_(rclcpp::QoS(5)),
  update_messages_received_(0),
  new_points_received_(false),
  color_factor_(0.8)
{
  connect(this, SIGNAL(mapUpdated()), this, SLOT(showMap()));

  update_topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Update Topic", "",
    "", "Topic where updates to this octomap display are received. "
    "This topic is automatically determined by the octomap topic. "
    "If the octomap is received on 'map_topic', the display assumes updates are received on "
    "'map_topic_updates'."
    "This can be overridden in the UI by clicking on the topic and setting the desired topic.",
    this, SLOT(updateMapUpdateTopic()));

  update_profile_property_ = new rviz_common::properties::QosProfileProperty(
    update_topic_property_, update_profile_);
  
  octree_render_property_ = new rviz_common::properties::EnumProperty( "Voxel Rendering", "Occupied Voxels",
                                                    "Select voxel type.",
                                                     this,
                                                     SLOT( updateOctreeRenderMode() ) );

  octree_render_property_->addOption( "Occupied Voxels",  OCTOMAP_OCCUPIED_VOXELS );
  octree_render_property_->addOption( "Free Voxels",  OCTOMAP_FREE_VOXELS );
  octree_render_property_->addOption( "All Voxels",  OCTOMAP_FREE_VOXELS | OCTOMAP_OCCUPIED_VOXELS);

  octree_coloring_property_ = new rviz_common::properties::EnumProperty( "Voxel Coloring", "Z-Axis",
                                                "Select voxel coloring mode",
                                                this,
                                                SLOT( updateOctreeColorMode() ) );

  octree_coloring_property_->addOption( "Cell Color",  OCTOMAP_CELL_COLOR );
  octree_coloring_property_->addOption( "Z-Axis",  OCTOMAP_Z_AXIS_COLOR );
  octree_coloring_property_->addOption( "Cell Probability",  OCTOMAP_PROBABLILTY_COLOR );
  alpha_property_ = new rviz_common::properties::FloatProperty( "Voxel Alpha", 1.0, "Set voxel transparency alpha",
                                             this, 
                                             SLOT( updateAlpha() ) );
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  tree_depth_property_ = new rviz_common::properties::IntProperty("Max. Octree Depth",
                                         max_octree_depth_,
                                         "Defines the maximum tree depth",
                                         this,
                                         SLOT (updateTreeDepth() ));
  tree_depth_property_->setMin(0);

  max_height_property_ = new rviz_common::properties::FloatProperty("Max. Height Display",
                                           std::numeric_limits<double>::infinity(),
                                           "Defines the maximum height to display",
                                           this,
                                           SLOT (updateMaxHeight() ));

  min_height_property_ = new rviz_common::properties::FloatProperty("Min. Height Display",
                                           -std::numeric_limits<double>::infinity(),
                                           "Defines the minimum height to display",
                                           this,
                                           SLOT (updateMinHeight() ));
}

OccupancyGridDisplay::~OccupancyGridDisplay()
{
  unsubscribe();
  clear();
}

OccupancyGridDisplay::OccupancyGridDisplay(rviz_common::DisplayContext * context)
: OccupancyGridDisplay()
{
  context_ = context;
  scene_manager_ = context->getSceneManager();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
}

void OccupancyGridDisplay::onInitialize()
{
  MFDClass::onInitialize();
  rviz_ros_node_ = context_->getRosNodeAbstraction();

  update_topic_property_->initialize(rviz_ros_node_);

  update_profile_property_->initialize(
    [this](rclcpp::QoS profile) {
      this->update_profile_ = profile;
      updateMapUpdateTopic();
    });

  box_size_.resize(max_octree_depth_);
  cloud_.resize(max_octree_depth_);
  point_buf_.resize(max_octree_depth_);
  new_points_.resize(max_octree_depth_);

  for (std::size_t i = 0; i < max_octree_depth_; ++i)
  {
    std::stringstream sname;
    sname << "PointCloud Nr." << i;
    cloud_[i] = new rviz_rendering::PointCloud();
    cloud_[i]->setName(sname.str());
    cloud_[i]->setRenderMode(rviz_rendering::PointCloud::RM_BOXES);
    scene_node_->attachObject(cloud_[i]);
  }
}

// method taken from octomap_server package
void OccupancyGridDisplay::setColor(double z_pos, double min_z, double max_z, double color_factor,
                                    rviz_rendering::PointCloud::Point& point)
{
  int i;
  double m, n, f;

  double s = 1.0;
  double v = 1.0;

  double h = (1.0 - std::min(std::max((z_pos - min_z) / (max_z - min_z), 0.0), 1.0)) * color_factor;

  h -= floor(h);
  h *= 6;
  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i)
  {
    case 6:
    case 0:
      point.setColor(v, n, m);
      break;
    case 1:
      point.setColor(n, v, m);
      break;
    case 2:
      point.setColor(m, v, n);
      break;
    case 3:
      point.setColor(m, n, v);
      break;
    case 4:
      point.setColor(n, m, v);
      break;
    case 5:
      point.setColor(v, m, n);
      break;
    default:
      point.setColor(1, 0.5, 0.5);
      break;
  }
}


void OccupancyGridDisplay::updateTreeDepth()
{
  updateTopic();
}

void OccupancyGridDisplay::updateOctreeRenderMode()
{
  updateTopic();
}

void OccupancyGridDisplay::updateOctreeColorMode()
{
  updateTopic();
}

void OccupancyGridDisplay::updateAlpha()
{
  updateTopic();
}

void OccupancyGridDisplay::updateMaxHeight()
{
  updateTopic();
}

void OccupancyGridDisplay::updateMinHeight()
{
  updateTopic();
}

void OccupancyGridDisplay::updateTopic()
{
  update_topic_property_->setValue(topic_property_->getTopic());
  MFDClass::updateTopic();
}

void OccupancyGridDisplay::subscribe()
{
  if (!isEnabled()) {
    return;
  }

  if (topic_property_->isEmpty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Topic",
      QString("Error subscribing: Empty topic name"));
    return;
  }

  MFDClass::subscribe();

  subscribeToUpdateTopic();
}

void OccupancyGridDisplay::subscribeToUpdateTopic()
{
  try {
    update_subscription_ =
      rviz_ros_node_.lock()->get_raw_node()->
      template create_subscription<octomap_msgs::msg::Octomap>(
      update_topic_property_->getTopicStd(), update_profile_,
      [this](const octomap_msgs::msg::Octomap::ConstSharedPtr message) {
        incomingUpdate(message);
      });
    setStatus(rviz_common::properties::StatusProperty::Ok, "Update Topic", "OK");
  } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Update Topic",
      QString("Error subscribing: ") + e.what());
  }
}

void OccupancyGridDisplay::unsubscribe()
{
  MFDClass::unsubscribe();
  unsubscribeToUpdateTopic();
}

void OccupancyGridDisplay::unsubscribeToUpdateTopic()
{
  update_subscription_.reset();
}

void OccupancyGridDisplay::clear()
{
  if (isEnabled()) {
    setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No octomap received");
  }

  if (!loaded_) {
    return;
  }

  loaded_ = false;
}

bool OccupancyGridDisplay::updateFromTF()
{
    // get tf transform
    Ogre::Vector3 pos;
    Ogre::Quaternion orient;
    if (!context_->getFrameManager()->getTransform(header_, pos, orient)) {
      return false;
    }

    scene_node_->setOrientation(orient);
    scene_node_->setPosition(pos);
    return true;
}

template <typename OcTreeType>
bool TemplatedOccupancyGridDisplay<OcTreeType>::checkType(std::string type_id)
{
  //General case: Need to be specialized for every used case
  setStatus(rviz_common::properties::StatusProperty::Warn, "Messages", QString("Cannot verify octomap type"));
  return true; //Try deserialization, might crash though
}

template <>
bool TemplatedOccupancyGridDisplay<octomap::OcTreeStamped>::checkType(std::string type_id)
{
  if(type_id == "OcTreeStamped") return true;
  else return false;
}
template <>
bool TemplatedOccupancyGridDisplay<octomap::OcTree>::checkType(std::string type_id)
{
  if(type_id == "OcTree") return true;
  else return false;
}

template <>
bool TemplatedOccupancyGridDisplay<octomap::ColorOcTree>::checkType(std::string type_id)
{
  if(type_id == "ColorOcTree") return true;
  else return false;
}

template <typename OcTreeType>
void TemplatedOccupancyGridDisplay<OcTreeType>::processMessage(octomap_msgs::msg::Octomap::ConstSharedPtr msg)
{
  current_map_ = *msg;
  loaded_ = true;
  // updated via signal in case ros spinner is in a different thread

  ++messages_received_;
  setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "Octomap received");
  setStatus(rviz_common::properties::StatusProperty::Ok, "Messages", QString::number(messages_received_) + " octomap messages received");
  setStatusStd(rviz_common::properties::StatusProperty::Ok, "Type", msg->id.c_str());
  if(!checkType(msg->id)){
    setStatusStd(rviz_common::properties::StatusProperty::Error, "Message", "Wrong octomap type. Use a different display type.");
    return;
  }

  header_ = msg->header;
  if (!updateFromTF()) {
      std::stringstream ss;
      ss << "Failed to transform from frame [" << header_.frame_id << "] to frame ["
          << context_->getFrameManager()->getFixedFrame() << "]";
      setStatusStd(rviz_common::properties::StatusProperty::Error, "Message", ss.str());
      return;
  }

  // creating octree
  OcTreeType* octomap = NULL;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
  if (tree){
    octomap = dynamic_cast<OcTreeType*>(tree);
    if(!octomap){
      setStatusStd(rviz_common::properties::StatusProperty::Error, "Message", "Wrong octomap type. Use a different display type.");
    }
  }
  else
  {
    setStatusStd(rviz_common::properties::StatusProperty::Error, "Message", "Failed to deserialize octree message.");
    return;
  }


  tree_depth_property_->setMax(octomap->getTreeDepth());

  // get dimensions of octree
  double minX, minY, minZ, maxX, maxY, maxZ;
  octomap->getMetricMin(minX, minY, minZ);
  octomap->getMetricMax(maxX, maxY, maxZ);

  // reset rviz pointcloud classes
  for (std::size_t i = 0; i < max_octree_depth_; ++i)
  {
    point_buf_[i].clear();
    box_size_[i] = octomap->getNodeSize(i + 1);
  }

  size_t pointCount = 0;
  {
    // traverse all leafs in the tree:
    unsigned int treeDepth = std::min<unsigned int>(tree_depth_property_->getInt(), octomap->getTreeDepth());
    double maxHeight = std::min<double>(max_height_property_->getFloat(), maxZ);
    double minHeight = std::max<double>(min_height_property_->getFloat(), minZ);
    int stepSize = 1 << (octomap->getTreeDepth() - treeDepth); // for pruning of occluded voxels
    for (typename OcTreeType::iterator it = octomap->begin(treeDepth), end = octomap->end(); it != end; ++it)
    {
        if(it.getZ() <= maxHeight && it.getZ() >= minHeight)
        {
          int render_mode_mask = octree_render_property_->getOptionInt();

          bool display_voxel = false;

          // the left part evaluates to 1 for free voxels and 2 for occupied voxels
          if (((int)octomap->isNodeOccupied(*it) + 1) & render_mode_mask)
          {
            // check if current voxel has neighbors on all sides -> no need to be displayed
            bool allNeighborsFound = true;

            octomap::OcTreeKey key;
            octomap::OcTreeKey nKey = it.getKey();

            // determine indices of potentially neighboring voxels for depths < maximum tree depth
            // +/-1 at maximum depth, +2^(depth_difference-1) and -2^(depth_difference-1)-1 on other depths
            int diffBase = (it.getDepth() < octomap->getTreeDepth()) ? 1 << (octomap->getTreeDepth() - it.getDepth() - 1) : 1;
            int diff[2] = {-((it.getDepth() == octomap->getTreeDepth()) ? diffBase : diffBase + 1), diffBase};

            // cells with adjacent faces can occlude a voxel, iterate over the cases x,y,z (idxCase) and +/- (diff)
            for (unsigned int idxCase = 0; idxCase < 3; ++idxCase)
            {
              int idx_0 = idxCase % 3;
              int idx_1 = (idxCase + 1) % 3;
              int idx_2 = (idxCase + 2) % 3;

              for (int i = 0; allNeighborsFound && i < 2; ++i)
              {
                key[idx_0] = nKey[idx_0] + diff[i];
                // if rendering is restricted to treeDepth < maximum tree depth inner nodes with distance stepSize can already occlude a voxel
                for (key[idx_1] = nKey[idx_1] + diff[0] + 1; allNeighborsFound && key[idx_1] < nKey[idx_1] + diff[1]; key[idx_1] += stepSize)
                {
                  for (key[idx_2] = nKey[idx_2] + diff[0] + 1; allNeighborsFound && key[idx_2] < nKey[idx_2] + diff[1]; key[idx_2] += stepSize)
                  {
                    typename OcTreeType::NodeType* node = octomap->search(key, treeDepth);

                    // the left part evaluates to 1 for free voxels and 2 for occupied voxels
                    if (!(node && ((((int)octomap->isNodeOccupied(node)) + 1) & render_mode_mask)))
                    {
                      // we do not have a neighbor => break!
                      allNeighborsFound = false;
                    }
                  }
                }
              }
            }

            display_voxel |= !allNeighborsFound;
          }


          if (display_voxel)
          {
            rviz_rendering::PointCloud::Point newPoint;

            newPoint.position.x = it.getX();
            newPoint.position.y = it.getY();
            newPoint.position.z = it.getZ();



            setVoxelColor(newPoint, *it, minZ, maxZ);
            // push to point vectors
            unsigned int depth = it.getDepth();
            point_buf_[depth - 1].push_back(newPoint);

            ++pointCount;
          }
        }
    }
  }

  if (pointCount)
  {
    new_points_received_ = true;

    for (size_t i = 0; i < max_octree_depth_; ++i)
      new_points_[i].swap(point_buf_[i]);

  }
  delete octomap;

  Q_EMIT mapUpdated();
}

void OccupancyGridDisplay::incomingUpdate(const octomap_msgs::msg::Octomap::ConstSharedPtr update)
{
  // Only update the octomap if we have gotten a full one first.
  if (!loaded_) {
    return;
  }

  ++update_messages_received_;
  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    "Topic",
    QString::number(update_messages_received_) + " update messages received");


  // updateMapDataInMemory(update);
  setStatus(rviz_common::properties::StatusProperty::Ok, "Update", "Update OK");

  // updated via signal in case ros spinner is in a different thread
  Q_EMIT mapUpdated();
}


void OccupancyGridDisplay::showMap()
{
  if (current_map_.data.empty()) {
    return;
  }


  showValidMap();
}

void OccupancyGridDisplay::showValidMap()
{
  frame_ = current_map_.header.frame_id;
  if (frame_.empty()) {
    frame_ = "/map";
  }

  setStatus(rviz_common::properties::StatusProperty::Ok, "Octomap", "Octomap OK");

  transformMap();

  context_->queueRender();
}


void OccupancyGridDisplay::transformMap()
{
  if (!loaded_) {
    return;
  }

  rclcpp::Time transform_time = context_->getClock()->now();

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  
  //if (!context_->getFrameManager()->transform(
  //    frame_, transform_time, current_map_.header.frame_id, position, orientation) &&
  //  !context_->getFrameManager()->transform(
  //    frame_, rclcpp::Time(0, 0, context_->getClock()->get_clock_type()),
  //    current_map_.header.frame_id, position, orientation))
  //{
  //  setMissingTransformToFixedFrame(frame_);
  //  scene_node_->setVisible(false);
  //} else {
  //  setTransformOk();
//
  //  scene_node_->setPosition(position);
  //  scene_node_->setOrientation(orientation);
  //}
}

void OccupancyGridDisplay::fixedFrameChanged()
{
  transformMap();
}

void OccupancyGridDisplay::reset()
{
  MFDClass::reset();
  update_messages_received_ = 0;
  clear();
}

void OccupancyGridDisplay::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;

  if (new_points_received_)
  {
    for (size_t i = 0; i < max_octree_depth_; ++i)
    {
      double size = box_size_[i];

      cloud_[i]->clear();
      cloud_[i]->setDimensions(size, size, size);
      cloud_[i]->addPoints(new_points_[i].begin(), new_points_[i].end());
      new_points_[i].clear();
      cloud_[i]->setAlpha(alpha_property_->getFloat());
    }
    new_points_received_ = false;
  }

  transformMap();
}

void OccupancyGridDisplay::onEnable()
{
  MFDClass::onEnable();
  setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No octomap received");
}

void OccupancyGridDisplay::updateMapUpdateTopic()
{
  unsubscribeToUpdateTopic();
  reset();
  subscribeToUpdateTopic();
  context_->queueRender();
}

template <typename OcTreeType>
void TemplatedOccupancyGridDisplay<OcTreeType>::setVoxelColor(rviz_rendering::PointCloud::Point& newPoint, 
                                                              typename OcTreeType::NodeType& node,
                                                              double minZ, double maxZ)
{
  OctreeVoxelColorMode octree_color_mode = static_cast<OctreeVoxelColorMode>(octree_coloring_property_->getOptionInt());
  float cell_probability;
  switch (octree_color_mode)
  {
    case OCTOMAP_CELL_COLOR:
      setStatus(rviz_common::properties::StatusProperty::Error, "Messages", QString("Cannot extract color"));
      //Intentional fall-through for else-case
    case OCTOMAP_Z_AXIS_COLOR:
      setColor(newPoint.position.z, minZ, maxZ, color_factor_, newPoint);
      break;
    case OCTOMAP_PROBABLILTY_COLOR:
      cell_probability = node.getOccupancy();
      newPoint.setColor((1.0f-cell_probability), cell_probability, 0.0);
      break;
    default:
      break;
  }
}

// Specialization for ColorOcTreeNode, which can set the voxel color from the node itself
template <>
void TemplatedOccupancyGridDisplay<octomap::ColorOcTree>::setVoxelColor(rviz_rendering::PointCloud::Point& newPoint, 
                                                                      octomap::ColorOcTree::NodeType& node,
                                                                      double minZ, double maxZ)
{
  float cell_probability;
  OctreeVoxelColorMode octree_color_mode = static_cast<OctreeVoxelColorMode>(octree_coloring_property_->getOptionInt());
  switch (octree_color_mode)
  {
    case OCTOMAP_CELL_COLOR:
    {
      const float b2f = 1./256.; 
      octomap::ColorOcTreeNode::Color& color = node.getColor();
      newPoint.setColor(b2f*color.r, b2f*color.g, b2f*color.b, node.getOccupancy());
      break;
    }
    case OCTOMAP_Z_AXIS_COLOR:
      setColor(newPoint.position.z, minZ, maxZ, color_factor_, newPoint);
      break;
    case OCTOMAP_PROBABLILTY_COLOR:
      cell_probability = node.getOccupancy();
      newPoint.setColor((1.0f-cell_probability), cell_probability, 0.0);
      break;
    default:
      break;
  }
}

}  // namespace octomap_rviz_plugin

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(octomap_rviz_plugin::OcTreeGridDisplay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(octomap_rviz_plugin::ColorOcTreeGridDisplay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(octomap_rviz_plugin::OcTreeStampedGridDisplay, rviz_common::Display)