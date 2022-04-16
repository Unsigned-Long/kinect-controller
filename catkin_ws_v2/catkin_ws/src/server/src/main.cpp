#include "camera_server.h"
#include "QTimer"
#include <QApplication>

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "server");
  ros::NodeHandle handler;

  QApplication a(argc, argv);

  Server s(handler, 3890, 9988);

  QEventLoop loop;
  ros::Rate r(30);

  while (ros::ok())
  {
    r.sleep();
    QTimer::singleShot(10, &loop, SLOT(quit()));
    loop.exec();
    ros::spinOnce();
  }

  return a.exec();
}
