//#include <QtGui/QApplication>
#include <QApplication>
#include "mainwindow.h"
#include <stdio.h>
#include "manager.h"
#include <GL/glut.h>
#include <string>
#include <iostream>

// #include "ros/ros.h"
//#include <std_msgs/String.h>
//#include <geometry_msgs/PoseStamped.h>
#include <rclcpp/rclcpp.hpp>
//#include <agent_msg/msg/registered_comp_image.hpp>
//#include <px4_msgs/msg/piksi_pos_llh.hpp>

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    QApplication a(argc, argv);
    a.setApplicationName("QHAC5");
    a.setQuitOnLastWindowClosed(true);

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    MainWindow w;
    w.show();

    return a.exec();
}
