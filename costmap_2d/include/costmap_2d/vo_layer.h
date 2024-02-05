#ifndef VO_LAYER_H_
#define VO_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <dynamic_reconfigure/server.h>

namespace vo_layer_namespace
{

class VOLayer : public costmap_2d::Layer
{
public:
  VOLayer();

  virtual void onInitialize();
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

protected:
  std::string global_frame_;

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  double mark_x_, mark_y_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif