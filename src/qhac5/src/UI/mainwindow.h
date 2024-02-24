#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "qcustomplot.h"
#include "ardronegraphicsitem.h"
#include "paramdialog.h"
#include "calibdialog.h"
#include "monitoringdialog.h"
#include "emscenariodialog.h"
#include "controldialog.h"
#include "filemanager.h"
#include "sendscdialog.h"
#include "qhac_mapview.h"

#include <QMainWindow>
#include <QGraphicsScene>
#include <QTimer>
#include <QThread>
#include <QLabel>
#include <QtMultimedia/QMediaPlayer>
#include <QLabel>
#include <QImage>
#include "opencv2/core/core_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/photo/photo.hpp"
#include <QWebView>
#include <QWebFrame>
#include <QWebElement>
#include <QRubberBand>
#include <QApplication>
#include <QGeoCoordinate>

#include <rclcpp/rclcpp.hpp>
// #include "ros/ros.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
// #include <mavros_msgs/RegisteredImage.h>
// #include <mavros_msgs/RegisteredCompImage.h>
// #include <mavros_msgs/PosLLH.h>
//#include <agent_msg/msg/registered_comp_image.hpp>
//#include <px4_msgs/msg/piksi_pos_llh.hpp>

namespace Ui {
    class MainWindow;
}


class CManager;
class CController;
class CScenario;

class CBallEstimator;

static const QGeoCoordinate   refPos(REF_LAT, REF_LON, REF_ALT);

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    virtual ~MainWindow();
    QImage cvMatToQImage( const cv::Mat &inMat );

public:
    bool                    processImage;
    QLabel                  *mImageLabel, *mInformationLabel, *mMapSelectionLabel;
    QGeoCoordinate getNewPositionDiff(QGeoCoordinate oldPosition, double x, double y, double z);
//    static const QGeoCoordinate   refPos;
    static QVector3D LLH2NED(QGeoCoordinate pos);
    static QGeoCoordinate NED2LLH(QVector3D pos);

protected:
    bool event(QEvent * event);
    void keyEvent(QKeyEvent * event);

private:
    void initManager();
    void subscribeROS2Topics();
    void procInitTreeWidget();
    void procInitMainPanelWidget();
    void updateTreeData();
    void updateDronesInMap();
    void updateStatusText();
    void updateNotifier();

private:    // ROS2 Topic
    rclcpp::Node::SharedPtr _ros2node;
//    rclcpp::Subscription<px4_msgs::msg::PiksiPosLLH>::SharedPtr _sub_piksi_posllh;
//    rclcpp::Subscription<agent_msg::msg::RegisteredCompImage>::SharedPtr _sub_regImg;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr _sub_agent_img;
//    void regiCompImageCallback(const agent_msg::msg::RegisteredCompImage::SharedPtr msg);
    void agentImgCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
//    void posLLHCallback(const px4_msgs::msg::PiksiPosLLH::SharedPtr msg);


private Q_SLOTS:
            void updateUI();
    void runScenario();
    void stopScenario();
    void loadConfigFile();
    void checkFlight();
    void runParamDialog();
    void runCalibration();
    void runMonitoringDialog();
    void onAlarm(bool aCheckable);
    void onControl();
    void onScenarioMode(bool aMode);
    void on_actionsendSC_triggered();

private slots:


private:
    Ui::MainWindow*         ui;
    QLabel*                 mRemaingTimeLabel;

    CParamDialog*           mParamDialog;
    CCalibDialog*           mCalibDialog;
    CMonitoringDialog*      mMonitorDialog;
    CEmScenarioDialog*      mEmScenarioDialog;
    CControlDialog*         mControlDialog;
    SendSCDialog*           mSendSCDialog;

    QGraphicsScene*         mMainPanelScene;

    QTimer                  mTimer;

    CManager*               mManager;
    CScenario*              mScenario;

    QThread                 mManagerThread;

    QMap<int, QString>      mPrevStatusText;

    QMediaPlayer            mAlarm;
    bool                    mReadyAlarm;

    const int               imageLabelWidth = 700;
    const int               imageLabelHeight = 700;
    QImage                  mQImage;
    QMutex                  mQImageMutex;
//    float                   mQImageX, mQImageY, mQImageWidth, mQImageHeight;
    QImage*                 mQRegisteredImage;
    QImage*                 mQInformationImage;
    QImage*                 mQMapSelectionImage;
    QPixmap                 mQPixmapImage, mQPixmapInformationImage, mQPixmapRegisteredImage, mQPixmapMapSelectionImage;
    qhac_mapview            *mMapView;
    QRubberBand             *mRubberBand;
    bool                    mRubberBandDrawing, mPolygonDrawing;
    QPoint                  mWindowPos;
    QList<QGeoCoordinate>   _base_info_list;
    QList<QGeoCoordinate>   _targetPositions;
    QGeoCoordinate          _base_latlng = QGeoCoordinate(36.374108, 127.352697, 82);

    float                   HEADING = 270;
    float                   TARGET_Z = 3;
    float                   DEPLOY_MINIMUM_DIST = 5.0;      // in meter
};

//MainWindow::refPos = QGeoCoordinate(REF_LAT, REF_LON, REF_ALT);

#endif // MAINWINDOW_H
