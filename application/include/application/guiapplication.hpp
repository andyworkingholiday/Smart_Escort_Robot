#ifndef GUIAPPLICATION_H
#define GUIAPPLICATION_H


/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_guiapplication.h"
#include "ShopType.hpp"
#include <iostream>
#include <QMouseEvent>
#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "qnode.hpp"
#include <QDialog>


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace application {

using namespace Qt;

class GuiApplication : public QDialog {
Q_OBJECT

public:
//    explicit GuiApplication(QWidget *parent = nullptr);
    ~GuiApplication();
    GuiApplication(int argc, char **argv, QWidget *parent = nullptr);
    void Init();

    //eventfilter for clicked qlabel(current, reload label)
    bool eventFilter(QObject *watched, QEvent* event);

    cv::Mat loadFromQrc(QString qrc, int flag = cv::IMREAD_COLOR);

    Ui::GuiApplication *ui;

private slots:
    void on_SearchButton_clicked();

    void on_Catesearch_clicked();

    void on_PathButton_clicked();

    void on_Showinfo_clicked();

    //SLOT of get path from qnode that recieve topic from astar algorithm
    void GetPath(const nav_msgs::Path::ConstPtr& msg);



private:
    ShopType shops[20];                                       //List of shop
    QNode qnode;                                              //for ros pub, sub
    geometry_msgs::PoseWithCovarianceStamped startpoint;      //start point of path (current location)
    geometry_msgs::PoseStamped targetPoint;                   //target point of path (destination shop)
    nav_msgs::OccupancyGrid occMap;
    cv::Mat map;
};

}  // namespace application

#endif // GUIAPPLICATION_H
