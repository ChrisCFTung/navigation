#ifndef VO_LAYER_H_
#define VO_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/VOPluginConfig.h>

#include <people_msgs/Person.h>
#include <people_msgs/People.h>
#include <dynamic_reconfigure/server.h>

namespace costmap_2d
{

class VOLayer : public CostmapLayer
{
public:
  VOLayer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
  }

  virtual void onInitialize();
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

protected:
  std::string global_frame_;
  void PoseCallback(const people_msgs::PeopleConstPtr& msg);
  ros::Subscriber sub_;
  double dt;
  int nstep;

private:
  void reconfigureCB(costmap_2d::VOPluginConfig &config, uint32_t level);
  std::vector<people_msgs::Person> poses;
  dynamic_reconfigure::Server<costmap_2d::VOPluginConfig> *dsrv_;
};
}
#endif