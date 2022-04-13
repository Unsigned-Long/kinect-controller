#include "camera_server.h"
#include "QDebug"
#include "QEventLoop"
#include "QNetworkInterface"
#include "QTimer"

Server::Server(ros::NodeHandle &handler, quint16 port, QObject *parent)
    : QObject(parent) {
  this->registerPublisher(handler);

  // create the server and socket
  this->createServer(port);

  // create connection
  this->connection();
}

Server::~Server() {
}

void Server::connection() {
  connect(this->_server, &QTcpServer::newConnection, this, [=]() {
    // when a new connection comming
    while (this->_server->hasPendingConnections()) {
      // connect
      this->_socket = this->_server->nextPendingConnection();

      // get info of this connection
      QString ip = this->_socket->peerAddress().toString();
      qint16 port = this->_socket->peerPort();

      ROS_INFO("'new connection': {'ip': '%s', 'port': '%d'}", ip.toStdString().c_str(), port);

      // read
      connect(this->_socket, &QTcpSocket::readyRead, this, [=]() {
        while (this->_socket->bytesAvailable()) {
          QByteArray buf = this->_socket->readAll();
          ROS_INFO("'server' gets message from 'client': '%s'", buf.toStdString().c_str());

          // handler message [start, stop]
          std_msgs::String msg;
          msg.data = buf.toStdString();
          this->_cameraController.publish(msg);
        }
      });
    }
  });
}

void Server::createServer(quint16 port) {
  // create the server and socket
  this->_server = new QTcpServer(this);
  this->_socket = new QTcpSocket(this);
  // listen
  this->_server->listen(QHostAddress::Any, port);

  // get ip address list
  QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();
  for (auto &elem : ipAddressesList) {
    ROS_INFO("'vaild ip': '%s'", elem.toString().toStdString().c_str());
  }
  ROS_INFO("'server port': '%d'", this->_server->serverPort());
  return;
}

void Server::sendState2Client(const std_msgs::String::ConstPtr &msg) {
  QEventLoop loop;

  ROS_INFO("'server' gets message from 'dk_camera': '%s'", msg->data.c_str());

  auto data = msg->data;
  // send to the client: color, depth, time
  if (this->_socket != nullptr) {
    this->_socket->write(QString::fromStdString(data).toUtf8());
  }
  
  QTimer::singleShot(1000, &loop, SLOT(quit()));
  loop.exec();
  return;
}

void Server::registerPublisher(ros::NodeHandle &handler) {
  // start, stop
  _cameraController = handler.advertise<std_msgs::String>("camera_control", 10);
  _stateChecker = handler.subscribe("runtime_state", 10, &Server::sendState2Client, this);
}