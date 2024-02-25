#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "manager.h"
#include "scenario.h"
#include "filemanager.h"

#include "sleeper.h"

#include <QKeyEvent>
#include <QFileDialog>
#include <QSignalMapper>
#include <QMessageBox>
#include <QPixmap>
#include <QImage>
#include <QtMath>

#include <opencv2/imgcodecs.hpp>
//#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // init alarm
    mReadyAlarm = false;

    mRemaingTimeLabel = new QLabel("--  ");
    QFont font = mRemaingTimeLabel->font();
    font.setPointSize(25);
    font.setBold(true);
    mRemaingTimeLabel->setFont(font);
    mRemaingTimeLabel->setAlignment(Qt::AlignCenter| Qt::AlignRight);
    mRemaingTimeLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    ui->mainToolBar->addWidget(mRemaingTimeLabel);

    mManager = new CManager();
    mScenario = new CScenario(mManager, this);
    mParamDialog = new CParamDialog(mManager, this);
    mCalibDialog = new CCalibDialog(mManager, this);
    mMonitorDialog = new CMonitoringDialog(mManager, this);
    mEmScenarioDialog = new CEmScenarioDialog(mManager, this);
    mControlDialog = new CControlDialog(mManager, this);
    mSendSCDialog = new SendSCDialog(mManager, this);

    connect(ui->mapView, SIGNAL(addStatusItem(const QString &)), this, SLOT(addStatusItem(const QString &)));
    ui->mapView->init(10,10);

    // FIXME: dynamic change according to the drone position
//    ui->mapView->moveByGPS(36.37501576001398, 127.35263774974969, 19);
    ui->mapView->moveByGPS(36.4537, 127.406, 19);


    mMapView = ui->mapView;
    mRubberBand = NULL;
    mRubberBandDrawing = false;
    mPolygonDrawing = false;


    //ui->mainWidget->setManager(mManager);

    mImageLabel = new QLabel(this);
    mInformationLabel = new QLabel(this);
    mMapSelectionLabel = new QLabel(this);
    mImageLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    mInformationLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    mMapSelectionLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    ui->gridLayout->addWidget(mImageLabel,0,0, Qt::AlignTop);
    ui->gridLayout->addWidget(mMapSelectionLabel,0,0, Qt::AlignTop);
    ui->gridLayout->addWidget(mInformationLabel,0,0, Qt::AlignTop);

    QPalette sample_palette;
    sample_palette.setColor(QPalette::WindowText, Qt::blue);

    mImageLabel->setPalette(sample_palette);

    mQRegisteredImage = nullptr;
    mQInformationImage = nullptr;
    mQMapSelectionImage = nullptr;

    this->subscribeROS2Topics();
}

MainWindow::~MainWindow()
{
    mManagerThread.quit();
    if(!mManagerThread.wait(3000)) //Wait until it actually has terminated (max. 3 sec)
    {
        mManagerThread.terminate(); //Thread didn't exit in time, probably deadlocked, terminate it!
        mManagerThread.wait(); //We have to wait again here!
    }

    delete mScenario;
    delete mManager;
    delete ui;
    if ( mParamDialog !=  NULL )	delete mParamDialog;
    if ( mCalibDialog !=  NULL )	delete mCalibDialog;
    delete mRemaingTimeLabel;
    if ( mEmScenarioDialog !=  NULL )	delete mEmScenarioDialog;
}

QImage MainWindow::cvMatToQImage( const cv::Mat &inMat )
{
    switch ( inMat.type() )
    {
        // 8-bit, 4 channel
        case CV_8UC4:
        {
            QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_ARGB32 );
            return image;
        }
            // 8-bit, 3 channel
        case CV_8UC3:
        {
            QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB888 );
            return image.rgbSwapped();
        }
            // 8-bit, 1 channel
        case CV_8UC1:
        {
            static QVector<QRgb>  sColorTable;

            // only create our color table once
            if ( sColorTable.isEmpty() )
            {
                for ( int i = 0; i < 256; ++i )
                    sColorTable.push_back( qRgb( i, i, i ) );
            }

            QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_Indexed8 );
            image.setColorTable( sColorTable );
            return image;
        }
        default:
//         ROS_WARN_STREAM("ImageDisplay::cvMatToQImage() - cv::Mat image type not handled in switch: " << inMat.type());
            break;
    }
    return QImage();
}

//void MainWindow::posLLHCallback(const px4_msgs::msg::PiksiPosLLH::SharedPtr msg)
//{
//    if (_base_info_list.size() < 30) {
//        QGeoCoordinate QGeoData = QGeoCoordinate(msg->lat, msg->lon, msg->height);
//        _base_info_list.push_back(QGeoData);
//    } else if (_base_info_list.size() == 30){
//        double lat_acc = 0, lon_acc = 0, alt_acc = 0;
//        QListIterator<QGeoCoordinate> qgeoIterator(_base_info_list);
//        while (qgeoIterator.hasNext()) {
//            QGeoCoordinate data = qgeoIterator.next();
//            lat_acc += data.latitude();
//            lon_acc += data.longitude();
//            alt_acc += data.altitude();
//        }
//        _base_latlng.setLatitude(lat_acc / _base_info_list.size());
//        _base_latlng.setLongitude(lon_acc / _base_info_list.size());
//        _base_latlng.setAltitude(alt_acc / _base_info_list.size());
//        qDebug() << "Base Position Fixed! " << _base_latlng;
//        _base_info_list.push_back(QGeoCoordinate(msg->lat, msg->lon, msg->height));
//    }
//}

//void MainWindow::regiCompImageCallback(const agent_msg::msg::RegisteredCompImage::SharedPtr msg)
//{
//    try {
//        cv::Mat image_mat = cv::imdecode(cv::Mat(msg->image.data),cv::IMREAD_UNCHANGED);
//        mQImage = QImage(cvMatToQImage(image_mat)).copy();
//
//        qDebug() << mQImage.size();
//        QVector3D pos = mManager->agent(2)->data("LLH").value<QVector3D>();
//        qreal z = -1.0 * mManager->agent(2)->data("POSZ").value<qreal>();
//        QGeoCoordinate llh = QGeoCoordinate(pos.x(), pos.y(), pos.z());
//        pos = LLH2NED(llh);
//        qDebug() << "pos :: " << pos << "alt : " << z;
//        qDebug() << "width: " << msg->width << msg->height;
//
//        // FOR KARI playground
////        qreal wm = 900;
////        qreal hm = 600;
////        qreal x_off = 68;
////        qreal y_off = 10;
////        qreal w_p = 2380;
////        qreal h_p = 1520;
//
//        // FOR Drone park
//        qreal wm = 1100+200;
//        qreal hm = 800+600;
//        qreal x_off = -110; // -110;
//        qreal y_off = 30;//90;
//        qreal w_p = 1280+1100;
//        qreal h_p = 720+800+800;    // add 800
//
//
////        qreal REAL_HFOV = qDegreesToRadians(46.0);      // 55 // 46
////        qreal REAL_VFOV = qDegreesToRadians(70.0);      // 55
//        qreal REAL_HFOV = qDegreesToRadians(74.4);
//        qreal REAL_VFOV = qDegreesToRadians(55.8);
//        qreal hd = 2.0 * z*qTan(REAL_HFOV/2.0);     // real horizontal distance (m)
//        qreal vd = 2.0 * z*qTan(REAL_VFOV/2.0);     // real vertical distance (m)
//        qreal px = pos.x() - (512+wm)*hd / 1024;
//        qreal py = pos.y() - (384+hm)*vd / 768;
//        qreal pz = z;
//        qreal w = w_p * (hd/1024);
//        qreal h = h_p * (vd/768);
//
//        QGeoCoordinate llh1 = QGeoCoordinate(NED2LLH(QVector3D(-px+x_off, py+y_off, pz)));
//        QGeoCoordinate llh2 = NED2LLH(LLH2NED(llh1) + QVector3D(w, h, 0));
//
//        qDebug() << "DIFF  : " << llh << llh1;
//
////        QGeoCoordinate llh1 = NED2LLH(QVector3D(msg->x, -msg->y, msg->height));
////        QGeoCoordinate llh2 = NED2LLH(QVector3D(msg->x + msg->width, -msg->y + msg->height, msg->height));
//
//        qreal width = llh1.latitude() - llh2.latitude();
//        qreal height = llh2.longitude() - llh1.longitude();
//
//        mMapView->updateImage(QRectF(llh1.latitude(),llh1.longitude(),width,height), mQImage);
//    } catch (cv_bridge::Exception& e) {
//        return;
//    } catch (...) {
//        printf("regiSmallCompImageCallback Exception!!\n");
//    }
//}

void MainWindow::agentImgCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    int target_id = 4;

    // read image. (original image is rorated (+90 degree))
    cv::Mat image_mat = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    transpose(image_mat, image_mat);
    flip(image_mat, image_mat, 0);
    mQImage = cvMatToQImage(image_mat);

    // calculate LLH and NED position
    QVector3D pos = mManager->agent(target_id)->data("LLH").value<QVector3D>();
    qreal z = -1.0 * mManager->agent(target_id)->data("POSZ").value<qreal>();
    QGeoCoordinate llh = QGeoCoordinate(pos.x(), pos.y(), z);
    pos = LLH2NED(llh);

    // calculate horizontal and vertical width and height distance (m) according to altitude
    qreal REAL_HFOV = qDegreesToRadians(74.4);
    qreal REAL_VFOV = qDegreesToRadians(55.8);
    qreal hd = 2.0 * z*qTan(REAL_HFOV/2.0);     // real horizontal distance (m)
    qreal vd = 2.0 * z*qTan(REAL_VFOV/2.0);     // real vertical distance (m)

    // calculate left-top position (NED -> LLH)
    pos.setX(pos.x() + 512*hd/1024);
    pos.setY(pos.y() - 384*vd/768);
    qreal width = hd;           // unit: meter
    qreal height = vd;          // unit: meter
    QGeoCoordinate llh1 = NED2LLH(pos);

    // calculate right-bottom poistion (NED -> LLH)
    QGeoCoordinate llh2 = NED2LLH(LLH2NED(llh1) + QVector3D(width, height, 0));

    // update image to mapview
    width = llh1.latitude() - llh2.latitude();
    height = llh2.longitude() - llh1.longitude();
    mMapView->updateImage(QRectF(llh1.latitude(),llh1.longitude(),width,height), mQImage);
}

void MainWindow::initManager()
{
    mManagerThread.setObjectName("Manager");
    connect(&mManagerThread, SIGNAL(started()), mManager, SLOT(onWork()));
    connect(&mManagerThread, SIGNAL(finished()), mManager, SLOT(onTerminated()));
    mManager->moveToThread(&mManagerThread);
    mManagerThread.start();
}

void MainWindow::subscribeROS2Topics()
{
    auto qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    _ros2node = rclcpp::Node::make_shared("qhac3node");
//    _sub_piksi_posllh = _ros2node->create_subscription<px4_msgs::msg::PiksiPosLLH>("/piksi_posllh", qos, std::bind(&MainWindow::posLLHCallback, this, _1));
//    _sub_regImg = _ros2node->create_subscription<agent_msg::msg::RegisteredCompImage>("/output/regiCompImage", qos, std::bind(&MainWindow::regiCompImageCallback, this, _1));
    _sub_agent_img = _ros2node->create_subscription<sensor_msgs::msg::CompressedImage>("/agent28/out/compressed", qos, std::bind(&MainWindow::agentImgCallback, this, _1));

}

void MainWindow::procInitTreeWidget()
{
    QStringList strItemList;

    strItemList << "MODE"
                << "READY_TO_FLY_FROM_MONITORING"
                << "Battery"
                << "RTK_STATUS"
                //				<< "RTK"
                << "MONITORING_STATUS1_HEX"
                << "LocalPos"
                //                << "Battery"  << "LocalVel" << "ATTITUDE"
                << "PX4_OSMO_BATT"
                //                << "RTK_TOW"
                << "SHIFT_STATE"
//                << "PARAM_STATUS"
            ;


    ui->treeWidget->setColumnCount(2);
    QStringList headers;
    headers << tr("Subject") << tr("Value");
    ui->treeWidget->setHeaderLabels(headers);
    ui->treeWidget->header()->resizeSection(0, 150);

    QList<QTreeWidgetItem *> items;
    int numItem = strItemList.size();
    for (int i = 0; i < numItem ; i++ ) {
        QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget);
        item->setText(0, strItemList[i]);
        item->setExpanded(true);

        const QMap<int, IAgent*> agentsMap = mManager->agents();
        QMap<int, IAgent*>::const_iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            QTreeWidgetItem *subitem = new QTreeWidgetItem(item);
            int sysid = agentsIterator.value()->data("SYSID").toInt();
            subitem->setText(0, QString("ID:%1[%2]").arg(agentsIterator.value()->id()).arg(sysid));
            subitem->setText(1, QString("---"));
            item->addChild(subitem);
        }

        items.append(item);

    }

    ui->treeWidget->insertTopLevelItems(0, items);

//    updateMapViewBounds();
    mQInformationImage = new QImage(mInformationLabel->width(), mInformationLabel->height(), QImage::Format_ARGB32);
    mQMapSelectionImage = new QImage(mInformationLabel->width(), mInformationLabel->height(), QImage::Format_ARGB32);
    mQRegisteredImage = new QImage(mImageLabel->width(), mImageLabel->height(), QImage::Format_ARGB32);
}

void MainWindow::procInitMainPanelWidget()
{

}

void MainWindow::updateTreeData()
{
    QMap<int, IAgent*> agentsMap = mManager->agents();

    for ( int i = 0 ; i < ui->treeWidget->topLevelItemCount() ; i++ ) {
        QTreeWidgetItem* item = ui->treeWidget->topLevelItem(i);

        int count = 0;
        QMap<int, IAgent*>::iterator agentsIterator;
        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
            QTreeWidgetItem* subitem = item->child(count);
            QString value;
            if ( agentsIterator.value() == NULL )  {
                qDebug("Error: agent == NULL");
                continue;
            }

            value = QString("%1").arg((agentsIterator.value()->data(item->text(0))).toString());

            // TEST
            if ( item->text(0) == "SHIFT_STATE" ) {
                value = mScenario->agentState(agentsIterator.value());
            }

            subitem->setText(1, value);

            if ( item->text(0) == "MONITORING_STATUS1_HEX" ) {
                QString tooltip = agentsIterator.value()->data("MONITORING_STR").toString();
                subitem->setToolTip(1, tooltip);
            }

            count++;
        }
    }
}

void MainWindow::updateDronesInMap()
{
    QMap<int, IAgent*> agentsMap = mManager->agents();
    QMap<int, IAgent*>::iterator agentsIterator;
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
        IAgent* agent = agentsIterator.value();
        QVector3D llh = agent->data("LLH").value<QVector3D>();
        float heading = agent->data("HEADING").value<qreal>();
        mMapView->updateDrone(agent->id(), llh.x(), llh.y(), heading);
    }
}

void MainWindow::updateStatusText()
{
    QMap<int, IAgent*> agentsMap = mManager->agents();
    QMap<int, IAgent*>::iterator agentsIterator;

    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
        int id = agentsIterator.value()->id();
        QString text = mManager->agent(id)->data("STATUSTEXT").toString();

        if ( !text.isEmpty() &&  text != mPrevStatusText[id] ) {
            QString statusText = QString("[%1] %2")
                    .arg(id)
                    .arg(text);

            ui->statusListWidget->addItem(statusText);
            ui->statusListWidget->scrollToBottom();
            mPrevStatusText[id] = mManager->agent(id)->data("STATUSTEXT").toString();
        }
    }
}

void MainWindow::updateNotifier()
{
    QMap<int, IAgent*> agentsMap = mManager->agents();
    bool ready = true;

    QMap<int, IAgent*>::const_iterator agentsIterator;
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
//    foreach (IAgent* agent, agentsMap) {
        int id = agentsIterator.value()->id();

        if ( mManager->agent(id)->data("RTK_READY") != "YES" ) {
            ready = false;
        }
    }

    if ( mReadyAlarm && ready && mAlarm.state() != QMediaPlayer::PlayingState ) {
        mAlarm.play();
        mReadyAlarm = false;
    }

}

void MainWindow::runScenario()
{
    QString scenario_name = mManager->property("emdscen", "name");
    ui->actionMode->setChecked(false);

    mScenario->start();
    /*
    if ( mScenario->isReady() && scenario_name == "" ) {
        qDebug("START SCENARIO");
        mScenario->start();
    }
    else {
        mEmScenarioDialog->startTimer();
        mEmScenarioDialog->show();
    }
    */
}

void MainWindow::stopScenario()
{
    mScenario->stop();
}

void MainWindow::loadConfigFile()
{
//    QString fileName = "/home/user/recon/qhac3_recon/Docs/CMODEL_4EA_2.conf";
    QString fileName = QFileDialog::getOpenFileName(
                this,
                tr("Open Agent Configuration File"),
                QString(CONFIG_FILE_PATH),
                tr("Conf Files (*.conf)"));


    if ( !fileName.isEmpty() ) {
        mManager->loadAgentFile(fileName);

        // init manager
        initManager();

        // wait for initializing manager thread
        // TODO: reduce sleep and check init Manager is finished.
        const QMap<int, IAgent*> agentsMap = mManager->agents();
        QMap<int, IAgent*>::const_iterator agentsIterator;
        bool isAllAgentsReady = true;
        do{
            CSleeper::msleep(500);
            isAllAgentsReady = true;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                if(agentsIterator.value()->isInitialized == false) {
                    isAllAgentsReady = false;
                    break;
                }
            }
//            qDebug() << "isAllAgentsReadey : " << isAllAgentsReady;
        } while(!isAllAgentsReady);

        procInitMainPanelWidget();
        procInitTreeWidget();
        connect(&mTimer, SIGNAL(timeout()), this, SLOT(updateUI()));
        mTimer.setInterval(33);
        mTimer.start();
    }

    _base_latlng.setLatitude(mManager->property("base", "latitude").toDouble());
    _base_latlng.setLongitude(mManager->property("base", "longitude").toDouble());
    _base_latlng.setAltitude(mManager->property("base", "altitude").toDouble());
    qDebug() << "set base! for SITL. " << _base_latlng;
}

void MainWindow::checkFlight()
{
    const QMap<int, IAgent*> agentsMap = mManager->agents();

    QMap<int, IAgent*>::const_iterator agentsIterator;
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
        agentsIterator.value()->cmd("RESET_PARAM");
        agentsIterator.value()->cmd("CHECK_PARAM");
    }
}

void MainWindow::runParamDialog()
{
    mParamDialog->updateNode();

    mParamDialog->show();
}

void MainWindow::runCalibration()
{
    mCalibDialog->initDialog();

    mCalibDialog->show();
}

void MainWindow::runMonitoringDialog()
{
    mMonitorDialog->startTimer();
    mMonitorDialog->show();
}

void MainWindow::onAlarm(bool aCheckable)
{
    if ( aCheckable == true ) {
        mReadyAlarm = aCheckable;
    }
    else {
        mAlarm.stop();
    }
}

void MainWindow::onControl()
{
    const QMap<int, IAgent*> agentsMap = mManager->agents();
    if (agentsMap.size() < 1) {
        QMessageBox msgBox;
        msgBox.setText("Open conf file first!");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    } else {
        mControlDialog->show();
    }
}

void MainWindow::onScenarioMode(bool aMode)
{
    QString cmdMode = "";
    if ( aMode == true ) {
        cmdMode = "OFF_EMBEDDED_SCENARIO";
    }
    else {
        cmdMode = "ON_EMBEDDED_SCENARIO";
    }

    const QMap<int, IAgent*> agentsMap = mManager->agents();
    QMap<int, IAgent*>::const_iterator agentsIterator;
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
//    foreach (IAgent* agent, agent_list) {
        agentsIterator.value()->cmd(cmdMode.toLatin1().data());
    }

}

void MainWindow::updateUI()
{
    updateTreeData();
    updateStatusText();
    //updateNotifier();
    //mRemaingTimeLabel->setText(mScenario->timeRemaining());
    rclcpp::spin_some(_ros2node);
    updateDronesInMap();
}

bool MainWindow::event(QEvent *event)
{
    if ( event->type() == QEvent::KeyRelease ){
        QKeyEvent *ke = static_cast<QKeyEvent *>(event);
        this->keyEvent(ke);
    }

    return QWidget::event(event);
}

QGeoCoordinate MainWindow::getNewPositionDiff(QGeoCoordinate oldPosition, double x, double y, double z)
{
    oldPosition = oldPosition.atDistanceAndAzimuth(x, 0, z);
    oldPosition = oldPosition.atDistanceAndAzimuth(y, 90);
    return oldPosition;
}

void MainWindow::on_actionsendSC_triggered()
{
    const QMap<int, IAgent*> agentsMap = mManager->agents();
    if (agentsMap.size() < 1) {
        QMessageBox msgBox;
        msgBox.setText("Open conf file first!");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    } else {
        mSendSCDialog->show();
    }
}

QVector3D MainWindow::LLH2NED(QGeoCoordinate pos)
{
    // Calc x,y,z of pos with refPos
    double NED_X = refPos.distanceTo(QGeoCoordinate(pos.latitude(), refPos.longitude(), refPos.altitude()));
    if (pos.latitude() < refPos.latitude())
        NED_X = -NED_X;
    double NED_Y = refPos.distanceTo(QGeoCoordinate(refPos.latitude(), pos.longitude(), refPos.altitude()));
    if (pos.longitude() < refPos.longitude())
        NED_Y = -NED_Y;
    double NED_Z = -(pos.altitude() - refPos.altitude());
    return QVector3D(NED_X, NED_Y, NED_Z);
}

QGeoCoordinate MainWindow::NED2LLH(QVector3D pos)
{
    // Calc lat, lon, alt of pos with refPos
    QGeoCoordinate LLHPosition = QGeoCoordinate(refPos.latitude(), refPos.longitude(), refPos.altitude());
    LLHPosition = LLHPosition.atDistanceAndAzimuth(pos.x(), 0, -pos.z());
    LLHPosition = LLHPosition.atDistanceAndAzimuth(pos.y(), 90);
    return LLHPosition;
}

void MainWindow::keyEvent(QKeyEvent *event)
{

    // find first agent
    int node = -1;
    const QMap<int, IAgent*> agentsMap = mManager->agents();
    QMap<int, IAgent*>::const_iterator agentsIterator;
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
        node = agentsIterator.value()->id();
        break;
    }
    if ( node < 0 ) return;

    QGeoCoordinate curPosition;

    switch(event->key()) {
        case Qt::Key_Q:
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();
                mManager->agent(node)->cmd("ARM", refPos.altitude());
            }

        }
            break;
        case Qt::Key_W:
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();
                mManager->agent(node)->cmd("DISARM");
            }

        }
            break;

        case Qt::Key_E:
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();
                mManager->agent(node)->cmd("LOCK");
            }
        }
            break;

        case Qt::Key_R:
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();
                mManager->agent(node)->cmd("UNLOCK");
            }
        }
            break;

        case Qt::Key_A:
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();
                mManager->agent(node)->cmd("TAKEOFF", TARGET_Z, HEADING);
            }
        }
            break;
        case Qt::Key_C:
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();
                mManager->agent(node)->cmd("AUTOMISSION");
            }
        }
            break;
        case Qt::Key_S:
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();
                mManager->agent(node)->cmd("LANDING", HEADING);
            }
        }
            break;

        case Qt::Key_D:
        {

        }
            break;
        case Qt::Key_F:
        {
            float startTime = mManager->agent(1)->data("RTK_TOW").toFloat() + 10.0;

            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();

                mManager->agent(node)->cmd("RESERVE_SCENARIO_TIME", startTime);
            }
        }
            break;
        case Qt::Key_G:
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();
                float posx = mManager->agent(node)->data("POSX").toDouble();
                float posy = mManager->agent(node)->data("POSY").toDouble();
                mManager->agent(node)->cmd("SET_SCENARIO_CONFS", posx, posy, 0, "default.sc");
            }
        }
            break;
        case Qt::Key_H:
        {
            float posx = mManager->agent(1)->data("POSX").toDouble();
            float posy = mManager->agent(1)->data("POSY").toDouble();

            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();
                QString scname = mManager->property("emdscen", "name");
                QString scfile = QString("%1/node_%2.txt").arg(scname).arg(node);
                mManager->agent(node)->cmd("SET_SCENARIO_CONFS", posx, posy, 0, scfile);
            }
        }
            break;
        case Qt::Key_M:
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();
                mManager->agent(node)->cmd("MANUAL");
            }
        }
            break;
        case Qt::Key_Z:
        {
//        const QMap<int, IAgent*> agentsMap = mManager->agents();
//        QMap<int, IAgent*>::const_iterator agentsIterator;
//        for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
//            node = agentsIterator.value()->id();
//             mManager->agent(node)->cmd("OFFBOARD");
//        }
            mManager->agent(3)->cmd("MANUAL");
        }
            break;
        case Qt::Key_O:
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                agentsIterator.value()->cmd("START_GST");
            }

        }
            break;
        case Qt::Key_P:
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                agentsIterator.value()->cmd("STOP_GST");
            }

        }
            break;
        case Qt::Key_1:
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();
                curPosition = QGeoCoordinate(mManager->agent(node)->data("GLOBAL_LAT").toDouble(), mManager->agent(node)->data("GLOBAL_LON").toDouble(), mManager->agent(node)->data("GLOBAL_ALT").toDouble());
                QVector3D curPosNED = LLH2NED(curPosition);
                printf("id : %d, x: %lf, y: %lf, z: %lf\n", node, curPosNED.x(), curPosNED.y(), curPosNED.z());
            }
        }
            break;
        case Qt::Key_2:
        {
            int NODE_ID = 5;

            QList<QVector3D> newPosList;
            newPosList.append(QVector3D(0, -14, -3));
            newPosList.append(QVector3D(5, -14, -3));
            newPosList.append(QVector3D(5, -19, -3));
            newPosList.append(QVector3D(0, -19, -3));
            QListIterator<QVector3D> iter(newPosList);
            while(iter.hasNext())
            {
                QGeoCoordinate newPositionGeo = NED2LLH(iter.next());
                mManager->agent(NODE_ID)->cmd("MOVE", newPositionGeo.latitude(), newPositionGeo.longitude(), newPositionGeo.altitude(), HEADING);
                CSleeper::msleep(10000);
            }
        }
            break;
        case Qt::Key_3:
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                agentsIterator.value()->cmd("SET_RESOLUTION_4K");
            }
        }
            break;

        case Qt::Key_4:
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                agentsIterator.value()->cmd("POSITION_SELFIE");
            }
        }
            break;

        case Qt::Key_5: // TEST
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                agentsIterator.value()->cmd("RECORD_START");
            }
        }
            break;

        case Qt::Key_6: // TEST
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                agentsIterator.value()->cmd("RECORD_STOP");
            }
        }
            break;
        case Qt::Key_7: // TEST
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            QList<QVector3D> newPosList;
            newPosList.append(QVector3D(4.65, -10.5, -15));
            newPosList.append(QVector3D(-1.0, -11.3, -15));
            newPosList.append(QVector3D(-2.1, -4.46, -15));
            newPosList.append(QVector3D(4.0, -4.21, -15));
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();
                QGeoCoordinate newPositionGeo = NED2LLH(newPosList.at(node-1));
//            printf("id : %d, newPosition lat : %lf, lon: %lf, alt: %lf\n", node, newPositionGeo.latitude(), newPositionGeo.longitude(), newPositionGeo.altitude());
                mManager->agent(node)->cmd("MOVE", newPositionGeo.latitude(), newPositionGeo.longitude(), newPositionGeo.altitude(), HEADING);
            }
        }
            break;
        case Qt::Key_8: // TEST
        {
//        TARGET_Z = TARGET_Z - 10;
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            QList<QVector3D> newPosList;
            newPosList.append(QVector3D(4.65, -10.5, -20));
            newPosList.append(QVector3D(-1.0, -11.3, -15));
            newPosList.append(QVector3D(-2.1, -4.46, -10));
            newPosList.append(QVector3D(4.0, -4.21, -13));
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();
                QGeoCoordinate newPositionGeo = NED2LLH(newPosList.at(node-1));
                mManager->agent(node)->cmd("MOVE", newPositionGeo.latitude(), newPositionGeo.longitude(), newPositionGeo.altitude(), HEADING);
            }
        }
            break;
        case Qt::Key_9:
        {
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            QList<QVector3D> newPosList;
            newPosList.append(QVector3D(0, -20, -60));
            newPosList.append(QVector3D(-55, -20, -60));
            newPosList.append(QVector3D(-55, 30, -60));
            newPosList.append(QVector3D(0, 30, -60));
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();
                QGeoCoordinate newPositionGeo = NED2LLH(newPosList.at(node-1));
                mManager->agent(node)->cmd("MOVE", newPositionGeo.latitude(), newPositionGeo.longitude(), newPositionGeo.altitude(), HEADING);
            }
        }
            break;

        case Qt::Key_0:
        {
            TARGET_Z = TARGET_Z + 10;
            const QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::const_iterator agentsIterator;
            QList<QVector3D> newPosList;
            newPosList.append(QVector3D(0, -20, -30));
            newPosList.append(QVector3D(-55, -20, -30));
            newPosList.append(QVector3D(-55, 30, -30));
            newPosList.append(QVector3D(0, 30, -30));
            for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator){
                node = agentsIterator.value()->id();
                QGeoCoordinate newPositionGeo = NED2LLH(newPosList.at(node-1));
                mManager->agent(node)->cmd("MOVE", newPositionGeo.latitude(), newPositionGeo.longitude(), newPositionGeo.altitude(), HEADING);
            }
        }
            break;

        default:
            break;
    };

    QMainWindow::keyPressEvent(event);
}
