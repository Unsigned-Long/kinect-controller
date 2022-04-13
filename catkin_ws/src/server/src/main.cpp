#include "camera_server.h"

#include <QApplication>

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "server");
  ros::NodeHandle handler;

  QApplication a(argc, argv);

  Server s(handler, 3890);
  
  ros::spin();
  return a.exec();
}
