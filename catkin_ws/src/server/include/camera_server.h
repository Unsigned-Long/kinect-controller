#ifndef SERVER_H
#define SERVER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <memory>

class Server : public QObject {
  // Q_OBJECT

public:
  Server(ros::NodeHandle &handler, quint16 port, QObject *parent = nullptr);
  ~Server();
  
  void sendState2Client(const std_msgs::String::ConstPtr &msg);

  void registerPublisher(ros::NodeHandle &handler);

  void createServer(quint16 port);

  void connection();

public:
  QTcpServer *_server = nullptr;
  QTcpSocket *_socket = nullptr;

  ros::NodeHandle _handler;
  ros::Publisher _cameraController;
  ros::Subscriber _stateChecker;
};

#endif // SERVER_H
