/**
 * @file /include/application/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef application_QNODE_HPP_
#define application_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <QThread>
#include <QStringListModel>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace application {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
  /*
  @brief: initialize ros node
  @pre: none
  @post: ros node is set
  */
	bool init();

  /*
  @brief: publish start point of path
  @pre: set start point
  @post: publish start point
  */
  void StartRun(geometry_msgs::PoseWithCovarianceStamped& msg);

  /*
  @brief: publish target point of path
  @pre: set start point
  @post: publish target point
  */
  void TargetRun(geometry_msgs::PoseStamped& msg);

  //PathHandler for Subscribe
  void PathHandler(const nav_msgs::Path::ConstPtr& msg);
  void MaskHandler(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  /*
  @brief: run ros::spin for subscribe
  @pre: none
  @post: ros::spin is execute in other thread
  */
  void run();

Q_SIGNALS:
  void rosShutdown();
  void SendPath(const nav_msgs::Path::ConstPtr& msg);

private:
  int init_argc;
  char** init_argv;
  ros::Publisher start_publisher;
  ros::Publisher target_publisher;
  ros::Subscriber path_sub;
  ros::Subscriber mask_sub;
  int cnt;


};

}  // namespace application

#endif /* application_QNODE_HPP_ */
