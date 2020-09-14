#include <QtGui>
#include <QMessageBox>
#include <QPixmap>
#include <QImage>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "../include/application/guiapplication.hpp"

namespace application {

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

//GuiApplication::GuiApplication(QWidget *parent) :
//    QDialog(parent),
//    ui(new Ui::GuiApplication)
//{
//    ui->setupUi(this);
//    Init();
//}

GuiApplication::GuiApplication(int argc, char **argv, QWidget *parent)
  : QDialog(parent), ui(new Ui::GuiApplication), qnode(argc, argv)
{
  ui->setupUi(this);
  Init();

  //regist MetaType for send parameter by topic
  qRegisterMetaType<nav_msgs::Path::ConstPtr>("nav_msgs::Path::ConstPtr");

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(SendPath(const nav_msgs::Path::ConstPtr&)), this, SLOT(GetPath(const nav_msgs::Path::ConstPtr&)));


  //initailize Location
  startpoint.pose.pose.position.x = 200.0;
  startpoint.pose.pose.position.y = 150.0;

  targetPoint.pose.position.x = 0.0;
  targetPoint.pose.position.y = 0.0;

  qnode.init();
}

GuiApplication::~GuiApplication()
{
    delete ui;
}

void GuiApplication::Init() {
    string name[10]={"0","Mcdonalds", "Starbucks","TGIfriday", "Dior", "Nike", "Twosome Place", "Chanel", "Adidas", "Toilet" };
    string pname[10]={"0", "mcdonalds.jpg", "starbucks.jpg", "friday.jpeg", "dior.jpeg", "nike.jpg", "twosome.jpg", "chanel.jpg", "adidas.jpeg", "toilet.jpeg"};
    int category[10]={-1, 1, 2, 1, 4, 3, 2, 4, 3, 5};
    cv::Point loc[10] = {cv::Point(0, 0), cv::Point(95, 150), cv::Point(90, 200), cv::Point(205, 205), cv::Point(317, 208),
                        cv::Point(320, 139), cv::Point(295, 205), cv::Point(319, 76), cv::Point(212, 72), cv::Point(95, 70)};

    for(int i=1; i<=9; i++){
        shops[i].SetRecord(i, category[i], name[i], pname[i]);
        shops[i].SetLocation(loc[i]);
    }
    const int w = ui->maplabel->width();
    const int h = ui->maplabel->height();
    ui->maplabel->setPixmap(QPixmap(":/maps/map.pgm").scaled(w,h,Qt::IgnoreAspectRatio));

    //add event of label
    ui->currentLoc_label->installEventFilter(this);
    ui->reload_label->installEventFilter(this);

    map = loadFromQrc(":/maps/map.pgm", CV_LOAD_IMAGE_COLOR);
}

}

void application::GuiApplication::GetPath(const nav_msgs::Path::ConstPtr& msg){
  cv::Mat pathmap = map.clone();

  int mapheight = pathmap.rows;
  int mapwidth = pathmap.cols;
  int mapch = pathmap.channels();

  ROS_INFO("get map from connect width: %d, height: %d, channel: %d", mapwidth, mapheight, mapch);

  cv::Point point;
  int cnt = 0;

  for(vector<geometry_msgs::PoseStamped>::const_iterator it = msg->poses.begin(); it != msg->poses.end();  ++it){
    cnt++;
    point.x = it->pose.position.x;
    point.y = it->pose.position.y;
    ROS_INFO("x : %d , y : %d\n", point.x, point.y);
    cv::line(pathmap, point, point, cv::Scalar(255, 0, 0), 2);
  }

  ROS_INFO("%d\n",cnt);
  ROS_INFO("Draw Successful");


  ui->maplabel->setPixmap(QPixmap::fromImage(QImage((unsigned char*) pathmap.data, pathmap.cols, pathmap.rows, QImage::Format_RGB888)));
}

//Load from Qt resource file for opencv
cv::Mat application::GuiApplication::loadFromQrc(QString qrc, int flag)
{
    //double tic = double(getTickCount());

    QFile file(qrc);
    cv::Mat m;
    if(file.open(QIODevice::ReadOnly))
    {
        qint64 sz = file.size();
        std::vector<uchar> buf(sz);
        file.read((char*)buf.data(), sz);
        m = cv::imdecode(buf, flag);
    }
    return m;
}


bool application::GuiApplication::eventFilter(QObject *watched, QEvent *event){
  if(watched == ui->reload_label && event->type() == QMouseEvent::MouseButtonPress){    //if reload is clicked
      map = loadFromQrc(":/maps/map.pgm", CV_LOAD_IMAGE_COLOR);
      const int w = ui->maplabel->width();
      const int h = ui->maplabel->height();
      ui->maplabel->setPixmap(QPixmap(":/maps/map.pgm").scaled(w,h,Qt::IgnoreAspectRatio));
      ROS_INFO("Reload Map");
  }else if(watched == ui->currentLoc_label && event->type() == QMouseEvent::MouseButtonPress){      //if current is clicked
      cv::Mat currentmap = loadFromQrc(":/maps/map.pgm", CV_LOAD_IMAGE_COLOR);
      cv::Point point;

      point.x = startpoint.pose.pose.position.x;
      point.y = startpoint.pose.pose.position.y;

      ROS_INFO("Currnt Position x: %d, y: %d", point.x, point.y);

      cv::line(currentmap, point, point, cv::Scalar(0, 100, 255), 5);

      ui->maplabel->setPixmap(QPixmap::fromImage(QImage((unsigned char*) currentmap.data, currentmap.cols, currentmap.rows, QImage::Format_RGB888)));
      map = currentmap;
  }

  return QDialog::eventFilter(watched, event);
}

void application::GuiApplication::on_SearchButton_clicked() {
    QString sname = ui->NameEdit->text();
    if(sname.isEmpty()){
      ROS_WARN("Text is empty");
      return ;
    }
    for(int i=1; i<=9; i++) {
        if(shops[i].GetName()==sname.toStdString()) {
            ui->shopnamelab->setText(sname);
            int catenum = shops[i].GetCategory();
            if(catenum==1) ui->Catelab->setText("Restaurant");
            else if(catenum==2) ui->Catelab->setText("Caffe");
            else if(catenum==3) ui->Catelab->setText("Clothes");
            else if(catenum==4) ui->Catelab->setText("Accesories");
            else if(catenum==5) ui->Catelab->setText("toilet");

            QString photopath, shoppath;
            photopath=":/shops/";
            shoppath=QString::fromStdString(shops[i].GetPhoto());
            photopath=photopath+shoppath;
            const int w = ui->Imagelabel->width();
            const int h = ui->Imagelabel->height();

            targetPoint.pose.position.x = shops[i].GetLocation().x;
            targetPoint.pose.position.y = shops[i].GetLocation().y;

            ui->Imagelabel->setPixmap(QPixmap(photopath).scaled(w,h,Qt::IgnoreAspectRatio));

            break;
        }
    }

}

void application::GuiApplication::on_Catesearch_clicked()
{
    ui->Shoptable->clearContents();
    QString cate = ui->comboBox->currentText();
    int icate;
    if(cate=="Restaurant") icate = 1;
    else if(cate=="Caffe") icate = 2;
    else if(cate=="Clothes") icate = 3;
    else if(cate=="Accesories") icate = 4;
    else icate = 5;

    int counts = 0;
    for(int i=1; i<=9; i++) {
        if(shops[i].GetCategory()==icate) {
            counts++;
            QString sname = QString::fromStdString(shops[i].GetName());
            QString snum = QString::number(shops[i].GetId());
            ui->Shoptable->setRowCount(counts);

            QTableWidgetItem* qsnum = new QTableWidgetItem(snum);
            QTableWidgetItem* qsname = new QTableWidgetItem(sname);
            qsnum->setTextAlignment(Qt::AlignCenter);
            qsname->setTextAlignment(Qt::AlignCenter);

            ui->Shoptable->setItem(counts-1, 0, qsnum);
            ui->Shoptable->setItem(counts-1, 1, qsname);
        }
    }
}

void application::GuiApplication::on_PathButton_clicked()
{
    /*find the location of the selected shop and make a map with the path */
    /* replace the map*/
    startpoint.header.stamp = ros::Time::now();
    startpoint.header.frame_id = "startPoint";

    targetPoint.header.stamp = ros::Time::now();
    targetPoint.header.frame_id = "targetPoint";

    qnode.StartRun(startpoint);
    qnode.TargetRun(targetPoint);
    ros::spinOnce();
}

void application::GuiApplication::on_Showinfo_clicked()
{
    int index = ui->Shoptable->currentRow();
    if(index == -1){
      ROS_WARN("Item is not selected");
      return ;
    }
    QTableWidgetItem *qsnum = ui->Shoptable->item(index, 0);
    QString snum = qsnum->text();
    int id = snum.toInt();
    for(int i=1; i<=9; i++) {
        if(shops[i].GetId()==id) {
            ui->tabWidget->setCurrentIndex(0);
            ui->shopnamelab->setText(QString::fromStdString(shops[i].GetName()));
            int catenum = shops[i].GetCategory();
            if(catenum==1) ui->Catelab->setText("Restaurant");
            else if(catenum==2) ui->Catelab->setText("Caffe");
            else if(catenum==3) ui->Catelab->setText("Clothes");
            else if(catenum==4) ui->Catelab->setText("Accesories");
            else if(catenum==5) ui->Catelab->setText("toilet");

            targetPoint.pose.position.x = shops[i].GetLocation().x;
            targetPoint.pose.position.y = shops[i].GetLocation().y;

            QString photopath, shoppath;
            photopath=":/shops/";
            shoppath=QString::fromStdString(shops[i].GetPhoto());
            photopath=photopath+shoppath;
            const int w = ui->Imagelabel->width();
            const int h = ui->Imagelabel->height();
            ui->Imagelabel->setPixmap(QPixmap(photopath).scaled(w,h,Qt::IgnoreAspectRatio));

            break;
        }
    }
}
