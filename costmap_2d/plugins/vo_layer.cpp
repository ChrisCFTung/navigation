#include <ros/ros.h>
#include <costmap_2d/vo_layer.h>
#include <tf2_ros/message_filter.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::VOLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace costmap_2d
{

// VOLayer::VOLayer() {}

void VOLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;
  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string topic;
  nh.param("topic", topic, std::string(""));

  sub_ = nh.subscribe(topic, 10, &VOLayer::PoseCallback, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::VOPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::VOPluginConfig>::CallbackType cb = boost::bind(
      &VOLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void VOLayer::PoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    // ROS_INFO("VO pose callback");
    mark_x_ = msg->pose.position.x;
    mark_y_ = msg->pose.position.y;
}

void VOLayer::reconfigureCB(costmap_2d::VOPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  combination_method_ = config.combination_method;
}

void VOLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
//   ROS_INFO("VO Layer Update begin");
  unsigned char* master_array = master_grid.getCharMap();
  unsigned int mx;
  unsigned int my;
//   ROS_INFO("obstacle coordinate: %f %f", mark_x_, mark_y_);
  bool do_update = master_grid.worldToMap(mark_x_, mark_y_, mx, my);
//   ROS_INFO("Performing Update: %d", do_update);
  if(do_update){
    // ROS_INFO("VO coor transform successful");
    unsigned int index = master_grid.getIndex(mx, my);
    // ROS_INFO("Updating cost for index %i", index);
    master_array[index] = LETHAL_OBSTACLE;
  }
}

} // end namespace