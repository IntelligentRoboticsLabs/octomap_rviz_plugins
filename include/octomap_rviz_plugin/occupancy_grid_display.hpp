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

#ifndef RVIZ_OCCUPANCY_GRID_DISPLAY_H
#define RVIZ_OCCUPANCY_GRID_DISPLAY_H

#include "rclcpp/rclcpp.hpp"
#include <mutex>
#include <message_filters/subscriber.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <std_msgs/msg/header.hpp>

#include <octomap/OcTreeStamped.h>
#include <octomap/ColorOcTree.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#ifndef Q_MOC_RUN 

#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <OgreSharedPtr.h>

#endif

//#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>

#include <rviz_common/display.hpp>
#include "rviz_common/display_context.hpp"
//#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz_rendering/objects/point_cloud.hpp"



namespace octomap_rviz_plugin
{

class OccupancyGridDisplay : public rviz_common::Display
{
Q_OBJECT
public:
  OccupancyGridDisplay();
  virtual ~OccupancyGridDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

private Q_SLOTS:
  void updateQueueSize();
  void updateTopic();
  void updateTreeDepth();
  void updateOctreeRenderMode();
  void updateOctreeColorMode();
  void updateAlpha();
  void updateMaxHeight();
  void updateMinHeight();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  void subscribe();
  void unsubscribe();

  virtual void incomingMessageCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) = 0;

  void setColor(
    double z_pos,
    double min_z,
    double max_z,
    double color_factor,
    rviz_rendering::PointCloud::Point& point);

  void clear();

  virtual bool updateFromTF();

  typedef std::vector<rviz_rendering::PointCloud::Point> VPoint;
  typedef std::vector<VPoint> VVPoint;
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_;

  std::mutex mutex_;

  // point buffer
  VVPoint new_points_;
  VVPoint point_buf_;
  bool new_points_received_;

  // Ogre-rviz point clouds
  std::vector<rviz_rendering::PointCloud*> cloud_;
  std::vector<double> box_size_;
  std_msgs::msg::Header header_;

  // Plugin properties
  std::shared_ptr<rviz_common::properties::IntProperty> queue_size_property_;
  std::shared_ptr<rviz_common::properties::RosTopicProperty> octomap_topic_property_;
  std::shared_ptr<rviz_common::properties::EnumProperty> octree_render_property_;
  std::shared_ptr<rviz_common::properties::EnumProperty> octree_coloring_property_;
  std::shared_ptr<rviz_common::properties::IntProperty> tree_depth_property_;
  std::shared_ptr<rviz_common::properties::FloatProperty> alpha_property_;
  std::shared_ptr<rviz_common::properties::FloatProperty> max_height_property_;
  std::shared_ptr<rviz_common::properties::FloatProperty> min_height_property_;

  u_int32_t queue_size_;
  uint32_t messages_received_;
  double color_factor_;
};

template <typename OcTreeType>
class TemplatedOccupancyGridDisplay: public OccupancyGridDisplay {
protected:
  void incomingMessageCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void setVoxelColor(rviz_rendering::PointCloud::Point& newPoint, typename OcTreeType::NodeType& node, double minZ, double maxZ);
  ///Returns false, if the type_id (of the message) does not correspond to the template paramter
  ///of this class, true if correct or unknown (i.e., no specialized method for that template).
  bool checkType(std::string type_id);
};

typedef octomap_rviz_plugin::TemplatedOccupancyGridDisplay<octomap::OcTree> OcTreeGridDisplay;
typedef octomap_rviz_plugin::TemplatedOccupancyGridDisplay<octomap::ColorOcTree> ColorOcTreeGridDisplay;
typedef octomap_rviz_plugin::TemplatedOccupancyGridDisplay<octomap::OcTreeStamped> OcTreeStampedGridDisplay;

} // namespace octomap_rviz_plugin

#endif //RVIZ_OCCUPANCY_GRID_DISPLAY_H
