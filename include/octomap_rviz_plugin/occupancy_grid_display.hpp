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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__MAP_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__MAP_DISPLAY_HPP_

#include <memory>
#include <string>
#include <vector>

#include <octomap/OcTreeStamped.h>
#include <octomap/ColorOcTree.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#ifndef Q_MOC_RUN

#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <OgreSharedPtr.h>

#endif  // Q_MOC_RUN

#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/qos.hpp"

#include "rviz_common/message_filter_display.hpp"

#include "rviz_default_plugins/visibility_control.hpp"
#include "rviz_rendering/objects/point_cloud.hpp"

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>

namespace Ogre
{
class ManualObject;
}

namespace rviz_common
{
namespace properties
{

class EnumProperty;
class FloatProperty;
class IntProperty;
class Property;
class QuaternionProperty;
class VectorProperty;

}  // namespace properties
}  // namespace rviz_common

namespace octomap_rviz_plugin
{

class AlphaSetter;


/**
 * \class OccupancyGridDisplay
 * \brief Displays a map along the XY plane.
 */
class OccupancyGridDisplay : public
  rviz_common::MessageFilterDisplay<octomap_msgs::msg::Octomap>
{
  Q_OBJECT

public:
  // TODO(botteroa-si): Constructor for testing, remove once ros_nodes can be mocked and call
  // initialize() instead
  explicit OccupancyGridDisplay(rviz_common::DisplayContext * context);
  OccupancyGridDisplay();
  ~OccupancyGridDisplay() override;

  void onInitialize() override;
  void fixedFrameChanged() override;
  void reset() override;

public Q_SLOTS:
  void showMap();

Q_SIGNALS:
  /** @brief Emitted when a new map is received*/
  void mapUpdated();

protected Q_SLOTS:
  /** @brief Show current_map_ in the scene. */
  void transformMap();
  void updateMapUpdateTopic();
  void updateQueueSize();
  void updateTreeDepth();
  void updateOctreeRenderMode();
  void updateOctreeColorMode();
  void updateAlpha();
  void updateMaxHeight();
  void updateMinHeight();

protected:
  void updateTopic() override;
  void update(float wall_dt, float ros_dt) override;

  void subscribe() override;
  void unsubscribe() override;

  void onEnable() override;

  /** @brief Copy update's data into current_map_ and call showMap(). */
  void incomingUpdate(octomap_msgs::msg::Octomap::ConstSharedPtr update);

  bool updateDataOutOfBounds(octomap_msgs::msg::Octomap::ConstSharedPtr update) const;
  void updateMapDataInMemory(octomap_msgs::msg::Octomap::ConstSharedPtr update);

  void clear();

  void subscribeToUpdateTopic();
  void unsubscribeToUpdateTopic();

  void showValidMap();

  bool updateFromTF();

  void setColor(
    double z_pos,
    double min_z,
    double max_z,
    double color_factor,
    rviz_rendering::PointCloud::Point& point);

  typedef std::vector<rviz_rendering::PointCloud::Point> VPoint;
  typedef std::vector<VPoint> VVPoint;
  // point buffer
  VVPoint new_points_;
  VVPoint point_buf_;
  bool new_points_received_;

  // Ogre-rviz point clouds
  std::vector<rviz_rendering::PointCloud*> cloud_;
  std::vector<double> box_size_;
  std_msgs::msg::Header header_;
  double color_factor_;

  bool loaded_;

  float resolution_;
  size_t width_;
  size_t height_;
  std::string frame_;
  octomap_msgs::msg::Octomap current_map_;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr update_subscription_;
  rclcpp::QoS update_profile_;

  rviz_common::properties::RosTopicProperty * update_topic_property_;
  rviz_common::properties::QosProfileProperty * update_profile_property_;
  rviz_common::properties::EnumProperty * octree_render_property_;
  rviz_common::properties::EnumProperty * octree_coloring_property_;
  rviz_common::properties::IntProperty * tree_depth_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * max_height_property_;
  rviz_common::properties::FloatProperty * min_height_property_;

  uint32_t update_messages_received_;
};

template <typename OcTreeType>
class RVIZ_DEFAULT_PLUGINS_PUBLIC TemplatedOccupancyGridDisplay : public OccupancyGridDisplay {
protected:
  /** @brief Copy msg into current_map_ and call showMap(). */
  void processMessage(octomap_msgs::msg::Octomap::ConstSharedPtr msg) override;
  
  void setVoxelColor(rviz_rendering::PointCloud::Point& newPoint, typename OcTreeType::NodeType& node, double minZ, double maxZ);
  ///Returns false, if the type_id (of the message) does not correspond to the template paramter
  ///of this class, true if correct or unknown (i.e., no specialized method for that template).
  bool checkType(std::string type_id);
};

typedef octomap_rviz_plugin::TemplatedOccupancyGridDisplay<octomap::OcTree> OcTreeGridDisplay;
typedef octomap_rviz_plugin::TemplatedOccupancyGridDisplay<octomap::ColorOcTree> ColorOcTreeGridDisplay;
typedef octomap_rviz_plugin::TemplatedOccupancyGridDisplay<octomap::OcTreeStamped> OcTreeStampedGridDisplay;

}  // namespace octomap_rviz_plugin

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__MAP_DISPLAY_HPP_
