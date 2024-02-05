#include <ros/ros.h>
#include <costmap_2d/vo_layer.h>
#include <tf2_ros/message_filter.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vo_layer_namespace::VOLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace vo_layer_namespace
{

VOLayer::VOLayer() {}

void VOLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;
  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string topic;
  nh.param("topic", topic, std::string(""));
  
  ros::Subscriber sub=nh.subscribe(topic, 10, PoseCallback);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &VOLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void VOLayer::PoseCallback(const geometry_msgs::PoseStampedConstPtr& msg,)
{
    mark_x_ = msg->pose->position->x;
    mark_y_ = msg->pose->position->y;
}

void VOLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void VOLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
  if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
}

} // end namespace