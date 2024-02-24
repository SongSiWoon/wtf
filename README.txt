====================================================
                  R E A D M E
====================================================

* Changed to ROS project (2019.04.12)
 - after clone the repository, execute following commands to run qhac3
 
 > source /opt/ros/kinetic/setup.bash
 > catkin clean
 > catkin_make
 > source devel/setup.bash
 > rosrun qhac3 qhac3

---------------------------------------------------------------------------



* MModel

  - 실외에서 동작하는 시스템



---------------
ETC

Agent("XXX")   <-- XXX는 각 Agent마다 다르게 분석할 수 있다.
 예를 들어
     AR.Drone은 "ID:IPAddr"
     Pixhawk는  "ID:/dev/ttyUSB0"



---------------

  CLogger logger("./test.log");
  logger.printf("XXXXXXX");
  logger << "XXXXX";

---------------

  mp3 player example
  http://qt-project.org/doc/qt-4.8/phonon-qmusicplayer.html
  http://qt-project.org/doc/qt-4.8/phonon-overview.html

---------------

* ardrone object xml
  <agentNum > 4 </agentNum>
  <agentInfo>
    <id> xx </id>
    <ip> xx.xx.xx.xx </ip>

* scenario xml
  <music> /home/xxxx/xxx.mp3 </music>
  <scenario>
    <time> 100 </time>
    <move> 1,2,3,4, </move>
    <ani> </ani>
