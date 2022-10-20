#include <QApplication>
#include <QIcon>
#include "camera_gui.h"


int main(int argc, char *argv[])
{

  ros::init(argc, argv, "camera_gui_node",ros::init_options::AnonymousName);
  QApplication a(argc, argv);

  CameraGui w;

  w.setWindowTitle(QString::fromStdString(ros::this_node::getName()));

  QIcon icon(":/icons/mistlogo.png");
  w.setWindowIcon(icon);

  w.show();
  return a.exec();
}
