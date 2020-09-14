/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/application/qnode.hpp"
#include <opencv2/opencv.hpp>


/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace cv;
using namespace std;
namespace application {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
  {cnt = 0;}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

void QNode::MaskHandler(const nav_msgs::OccupancyGrid::ConstPtr& msg){       //function for get mask of map
  ROS_INFO("Get Mask");
}

void QNode::PathHandler(const nav_msgs::Path::ConstPtr& msg){                //function for get path of navigation from astar
  if(cnt == 0){
    ROS_INFO("Get Path");
    cnt++;
    Q_EMIT SendPath(msg);
  }
}

bool QNode::init() {                                        //initailize ros and subscribe
  ros::init(init_argc,init_argv,"application");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	// Add your ros communications here.

  ros::NodeHandle nh;

  start_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/application/start_loc", 5);
  target_publisher = nh.advertise<geometry_msgs::PoseStamped>("/application/target_loc", 5);


  path_sub = nh.subscribe("/move_base/NavfnROS/plan", 100, &QNode::PathHandler, this);
  mask_sub = nh.subscribe("/escort_core/map_mask", 100, &QNode::MaskHandler, this);
	start();

	return true;
}

void QNode::run() {
  ros::Rate loop_rate(100);
  while ( ros::ok() ) {

    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::StartRun(geometry_msgs::PoseWithCovarianceStamped& msg) {           //publish start point
    ROS_INFO("Publish StartLocation at application");
    start_publisher.publish(msg);
}

void QNode::TargetRun(geometry_msgs::PoseStamped &msg){  //publish target point
  ROS_INFO("Publish TargetLocation at application");
  cnt = 0;
  target_publisher.publish(msg);
}

}  // namespace application
