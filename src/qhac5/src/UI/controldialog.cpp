#include "controldialog.h"
#include "ui_controldialog.h"
#include "manager.h"
#include "agent.h"
#include <QtMath>
#include <QApplication>
#include "mainwindow.h"

CControlDialog::CControlDialog(CManager *aManager, QWidget *parent) :
        QDialog(parent),
        ui(new Ui::CControlDialog) {
    ui->setupUi(this);
    this->setWindowTitle("Control Dialog");

    mNumAgent = 0;
    mNodeButton = NULL;
    mNodeLabel = NULL;
    mSignalMapper = new QSignalMapper(this);

    mManager = aManager;

    connect(&mTimer, SIGNAL(timeout()), this, SLOT(updateStatus()));
    mTimer.setInterval(1000);

    ocmTimer = new QTimer(this);
    connect(ocmTimer, &QTimer::timeout, this, &CControlDialog::sendOCMInterval);

    connect(ui->type_comboBox, SIGNAL(currentTextChanged(QString)),
            this, SLOT(updateTypeCombobox(QString)));
    ui->offboardButton->hide();
}

CControlDialog::~CControlDialog() {
    if (mNodeButton != NULL) delete mNodeButton;
    if (mNodeLabel != NULL) delete mNodeLabel;
    if (mSignalMapper != NULL) delete mSignalMapper;

    delete ui;
}

void CControlDialog::initNodeLayout() {
    mNumAgent = mManager->numOfAgent();

    // free allocated memory
    if (mNodeButton != NULL) delete mNodeButton;
    if (mNodeLabel != NULL) delete mNodeLabel;
    disconnect(mSignalMapper, 0, 0, 0);


    // create button and label
    mNodeButton = new QPushButton *[mNumAgent];
    mNodeLabel = new QLabel *[mNumAgent];

    // set combobox list
    QStringList comboboxItemList;
    comboboxItemList << "REBOOT" << "OFFBOARD" << "ARM" << "TAKEOFF" << "MOVE" << "LANDING" << "DISARM";

    int i = 0;
    QMap < int, IAgent * > agentsMap = mManager->agents();
    QMap<int, IAgent *>::iterator agentsIterator;

    // get max id number
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator) {
//    foreach (IAgent* agent, agentsMap) {
        int id = agentsIterator.value()->id();
        if (id > maxId) {
            maxId = id;
        }
    }

    // Initialize variables for auto deploy
    currentStatus = new int *[maxId + 1];
    retryCount = new int *[maxId + 1];
    currentX = new float *[maxId + 1];
    currentY = new float *[maxId + 1];
    currentZ = new float *[maxId + 1];
    targetX = new float *[maxId + 1];
    targetY = new float *[maxId + 1];
    prevX = new float **[maxId + 1];
    prevY = new float **[maxId + 1];
    prevZ = new float **[maxId + 1];
    prevIndex = new int *[maxId + 1];

    float offx = mManager->agent(1)->data("POSX").toDouble();
    float offy = mManager->agent(1)->data("POSY").toDouble();

    for (int i = 0; i <= maxId; i++) {
        currentStatus[i] = new int;
        *currentStatus[i] = 0;

        retryCount[i] = new int;
        *retryCount[i] = 0;
    }

    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator) {
//    foreach (IAgent* agent, agentsMap) {
        int agentId = agentsIterator.value()->id();

        currentX[agentId] = new float;
        currentY[agentId] = new float;
        currentZ[agentId] = new float;

        targetX[agentId] = new float;
        targetY[agentId] = new float;

        prevX[agentId] = new float *[TIME_TO_STABLIZE];
        prevY[agentId] = new float *[TIME_TO_STABLIZE];
        prevZ[agentId] = new float *[TIME_TO_STABLIZE];
        for (int prevIndex = 0; prevIndex < TIME_TO_STABLIZE; prevIndex++) {
            prevX[agentId][prevIndex] = new float;
            prevY[agentId][prevIndex] = new float;
            prevZ[agentId][prevIndex] = new float;

            *prevX[agentId][prevIndex] = -99999;
            *prevY[agentId][prevIndex] = -99999;
            *prevZ[agentId][prevIndex] = -99999;
        }
        prevIndex[agentId] = new int;
        *prevIndex[agentId] = 0;

        *targetX[agentId] = offx + ((agentId - 1) % COLUMN_NODE) * INTERVAL;
        *targetY[agentId] = offy - ((agentId - 1) / COLUMN_NODE) * INTERVAL;
    }


//    foreach (IAgent* agent, agentsMap) {
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator) {
        int agentId = agentsIterator.value()->id();
        *currentX[agentId] = agentsIterator.value()->data("POSX").toFloat();
        *currentY[agentId] = agentsIterator.value()->data("POSY").toFloat();
        *currentZ[agentId] = -agentsIterator.value()->data("POSZ").toFloat();
    }

    // create agent button
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator) {
        int id = agentsIterator.value()->id();
        int sysid = agentsIterator.value()->data("SYSID").toInt();

        // create button
        mNodeButton[i] = new QPushButton(this);
        mNodeButton[i]->setFixedSize(70, 40);
        mNodeButton[i]->setStyleSheet("font-size: 17px;font: bold");
        //mNodeButton[i]->setText("--");
        QString name = QString("C%1\n(%2)").arg(sysid).arg(id);
        mNodeButton[i]->setText(name);

        mNodeButton[i]->setCheckable(true);

        // create label
        mNodeLabel[i] = new QLabel(this);
        mNodeLabel[i]->setFixedSize(90, 10);
        mNodeLabel[i]->setWordWrap(true);
        mNodeLabel[i]->setText(" ");

        // add widget to layout
        int tableMaxNum = (qCeil((float) maxId / COLUMN_NODE)) * COLUMN_NODE;
        int row = (tableMaxNum - id) / COLUMN_NODE;
        int col = (id - 1) % COLUMN_NODE;
        ui->node_gridLayout->addWidget(mNodeButton[i], row * 3 + 1, col, 1, 1);
        ui->node_gridLayout->addWidget(mNodeLabel[i], row * 3 + 2, col, 1, 1);

        // set alignment
        ui->node_gridLayout->setAlignment(mNodeButton[i], Qt::AlignHCenter);
        ui->node_gridLayout->setAlignment(mNodeLabel[i], Qt::AlignHCenter);


        // Mapping
        mButtonIndexMap[id] = i;

        // connection
        connect(mNodeButton[i], &QPushButton::clicked, [this, id, i]() {
            if (mNodeButton[i]->isChecked()) {
                selectNodeList.append(id);
                mNodeButton[i]->setStyleSheet("QPushButton {"
                                              "font-size: 17px;"
                                              "font: bold;"
                                              "background-color: #4CAF50;"
                                              "color: white;"
                                              "}");
            } else {
                selectNodeList.removeAll(id);
                mNodeButton[i]->setStyleSheet("QPushButton { font-size: 17px; font: bold; }");
            }
        });

//        connect (mNodeButton[i], SIGNAL(clicked(bool)), mSignalMapper, SLOT(map())) ;
//        mSignalMapper->setMapping(mNodeButton[i], id);

        i++;
    }

//    connect(mSignalMapper, SIGNAL(mapped(int)), this, SLOT(onNodeClicked(int)));
}

//void CControlDialog::onNodeClicked(int aId) {
//    QString type = ui->type_comboBox->currentText();
//
//    if (type == "Control") {
//        currentSelectedNode = aId;
//        QString selectedNodeText = QString("C%1(%2)").arg(mManager->agent(aId)->data("SYSID").toInt()).arg(aId);
//    } else if (type == "Emergency") {
//        mManager->agent(aId)->cmd("MANUAL");
//    } else {
//        qDebug("ERR: Unknown type");
//    }
//}

void CControlDialog::onAllLand() {
    const QMap<int, IAgent *> agentsMap = mManager->agents();
    QMap<int, IAgent *>::const_iterator agentsIterator;
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator) {
        int node = agentsIterator.value()->id();
        mManager->agent(node)->cmd("LANDING", 0);
    }
}

void CControlDialog::updateStatus() {
    QString normalStyle = "font-size:17px;font:bold;";
    QString readyToFlyStyle = "font-size:17px;font: bold;";
    QString emergencyStyle = normalStyle + QString("background-color: rgb(200,0,0);");

    QMap < int, IAgent * > agentsMap = mManager->agents();
    QMap<int, IAgent *>::iterator agentsIterator;
    for (agentsIterator = agentsMap.begin(); agentsIterator != agentsMap.end(); ++agentsIterator) {
        int id = agentsIterator.value()->id();
        int sysid = agentsIterator.value()->data("SYSID").toInt();
        int index = mButtonIndexMap[id];

        QString name = QString("C%1\n(%2)").arg(sysid).arg(id);
        mNodeButton[index]->setText(name);
#if 0
        if ( agentsIterator.value()->data("IS_ARMED").toBool() == false ) {
            QString rtf_result = agentsIterator.value()->data("READY_TO_FLY_FROM_MONITORING").toString();
            if ( rtf_result == "OK" || rtf_result == "OK(ON)" ) {
                mNodeButton[index]->setStyleSheet(readyToFlyStyle);
                mNodeLabel[index]->setText("");
            }
            else {
                mNodeButton[index]->setStyleSheet(emergencyStyle);
                mNodeLabel[index]->setText(rtf_result);
            }
        }
        else {
            QString emer_result = agentsIterator.value()->data("EMERGENCY_FLIGHT_FROM_MONITORING").toString();
            if ( emer_result == "NO" ) {
                mNodeButton[index]->setStyleSheet(normalStyle);
                mNodeLabel[index]->setText("");
            }
            else {
                mNodeButton[index]->setStyleSheet(emergencyStyle);
                mNodeLabel[index]->setText(emer_result);
            }
        }
#endif
    }
    QString allNodesPosText;

    for (int nodeId: selectNodeList) {
        double curLat = mManager->agent(nodeId)->data("GLOBAL_LAT").toDouble();
        double curLon = mManager->agent(nodeId)->data("GLOBAL_LON").toDouble();
        double curAlt = mManager->agent(nodeId)->data("GLOBAL_ALT").toDouble();
        QVector3D curPosQVector = mManager->agent(nodeId)->data("POS").value<QVector3D>();
        QVector3D refLLH = mManager->agent(nodeId)->data("REF_LLH").value<QVector3D>();

        QString curPosText = QString("Node %1: L(%2, %3, %4) G(%5, %6, %7) REF(%8, %9, %10)")
                .arg(nodeId, -3)
                .arg(QString::number(curPosQVector.x(), 'f', 3), 8)
                .arg(QString::number(curPosQVector.y(), 'f', 3), 8)
                .arg(QString::number(curPosQVector.z(), 'f', 3), 8)
                .arg(QString::number(curLat, 'f', 6), 8)
                .arg(QString::number(curLon, 'f', 6), 8)
                .arg(QString::number(curAlt, 'f', 3), 8)
                .arg(QString::number(refLLH.x(), 'f', 6), 8)
                .arg(QString::number(refLLH.y(), 'f', 6), 8)
                .arg(QString::number(refLLH.z(), 'f', 3), 8);

        allNodesPosText += curPosText + "\n";
    }

    if (!allNodesPosText.isEmpty()) {
        ui->curPosLabel->setStyleSheet("font-size: 10pt;");
        ui->curPosLabel->setText(allNodesPosText);
    } else {
        ui->curPosLabel->setStyleSheet("font-size: 20pt; font: bold;");
        ui->curPosLabel->setText("No nodes selected.");
    }
}

void CControlDialog::changeButtonColor(int agentId, QColor color) {
    int buttonIndex = mButtonIndexMap[agentId];
    QPalette pal = mNodeButton[buttonIndex]->palette();
    pal.setColor(QPalette::Button, color);
    mNodeButton[buttonIndex]->setAutoFillBackground(true);
    mNodeButton[buttonIndex]->setPalette(pal);
    mNodeButton[buttonIndex]->update();
}

bool CControlDialog::shouldRetry(int agentId) {
    if (*retryCount[agentId] <= MAX_RETRY_COUNT) {
        *retryCount[agentId] = *retryCount[agentId] + 1;
        changeButtonColor(agentId, Qt::yellow);
        return true;
    }
    changeButtonColor(agentId, Qt::red);
    return false;
}

void CControlDialog::addNextAgentToQueue(int agentId) {
    int nextAgentId = agentId + 1;
    while (nextAgentId <= maxId && !mManager->hasAgent(nextAgentId) && *currentStatus[nextAgentId] == STATUS_INITIAL) {
        nextAgentId++;
    }

    if (nextAgentId <= maxId) {
//        qDebug("addNextAgentToQueue : %d",nextAgentId);
        queueToFlight.enqueue(nextAgentId);
    }
}

float CControlDialog::targetXYDistance(int mAgentId) {
    float distX = *targetX[mAgentId] - *currentX[mAgentId];
    float distY = *targetY[mAgentId] - *currentY[mAgentId];

    return sqrtf(powf(distX, 2) + powf(distY, 2));
}

void CControlDialog::updateTypeCombobox(QString aType) {
    if (aType == "Offboard") {
        ui->offboardButton->show();
        ocmTimer->start(400);
    } else {
        ui->offboardButton->hide();
        ocmTimer->stop();
    }
}

void CControlDialog::showEvent(QShowEvent *event) {
    Q_UNUSED(event);

    this->move(500, 200);
    if (mNumAgent == 0) {
        initNodeLayout();
    }
    mTimer.start();
}

void CControlDialog::closeEvent(QCloseEvent *event) {
    Q_UNUSED(event);
    mTimer.stop();
}

void CControlDialog::hideEvent(QHideEvent *event) {
    Q_UNUSED(event);
    mTimer.stop();
}

void CControlDialog::addHistory(QString text, int aId) {
    int sysid = mManager->agent(aId)->data("SYSID").toInt();
    QDateTime dateTime = dateTime.currentDateTime();
    QString controlText = QString("[%1] C%2[%3] %4")
            .arg(dateTime.toString("yyyy-MM-dd HH:mm:ss")).arg(sysid).arg(aId)
            .arg(text);
    ui->controlHistoryWidget->addItem(controlText);
    ui->controlHistoryWidget->scrollToBottom();
}

void CControlDialog::on_resetLposButton_clicked()
{
    if (selectNodeList.isEmpty()) {
        qDebug() << "Select Drone(s) First!";
        return;
    }

    for (int nodeId: selectNodeList) {
        mManager->agent(nodeId)->cmd("RESET_LPOS");
        addHistory(QString("RESET_LPOS %1").arg(nodeId), nodeId);
    }
}

void CControlDialog::on_rebootButton_clicked()
{
    if (selectNodeList.isEmpty()) {
        qDebug() << "Select Drone(s) First!";
        return;
    }

    for (int nodeId: selectNodeList) {
        mManager->agent(nodeId)->cmd("REBOOT");
        addHistory(QString("REBOOT %1").arg(nodeId), nodeId);
    }
}

void CControlDialog::on_offboardButton_clicked()
{
    if (selectNodeList.isEmpty()) {
        qDebug() << "Select Drone(s) First!";
        return;
    }

    for (int nodeId: selectNodeList) {
        double heading = mManager->agent(nodeId)->data("HEADING").toDouble();

        sendCommand(nodeId, QVector3D(0.0, 0.0, 0.0), heading);

        mManager->agent(nodeId)->cmd("OFFBOARD");
        addHistory(QString("OFFBOARD %1").arg(nodeId), nodeId);
    }
}

void CControlDialog::on_armButton_clicked() {
    if (selectNodeList.isEmpty()) {
        qDebug() << "Select Drone(s) First!";
        return;
    }

    for (int nodeId: selectNodeList) {
        double curAlt = mManager->agent(nodeId)->data("GLOBAL_ALT").toDouble();
        mManager->agent(nodeId)->cmd("ARM", curAlt);
        addHistory(QString("ARM %1").arg(nodeId), nodeId);
    }
}


void CControlDialog::on_takeoffButton_clicked() {
    if (selectNodeList.isEmpty()) {
        qDebug() << "Select Drone(s) First!";
        return;
    }

    for (int nodeId: selectNodeList) {
        int headingSetpoint = ui->headingSpinBox->value();
        mManager->agent(nodeId)->cmd("TAKEOFF", 3.0);
        addHistory("TAKE-OFF", nodeId);
    }
}


void CControlDialog::on_landButton_clicked() {
    if (selectNodeList.isEmpty()) {
        qDebug() << "Select Drone(s) First!";
        return;
    }

    for (int nodeId: selectNodeList) {
        int headingSetpoint = ui->headingSpinBox->value();
        mManager->agent(nodeId)->cmd("LANDING", headingSetpoint);
        addHistory("LAND", nodeId);
    }
}


void CControlDialog::on_disarmButton_clicked() {
    if (selectNodeList.isEmpty()) {
        qDebug() << "Select Drone(s) First!";
        return;
    }

    for (int nodeId: selectNodeList) {
        mManager->agent(nodeId)->cmd("DISARM");
        addHistory("DISARM", nodeId);
    }
}


void CControlDialog::on_upButton_clicked() {
    if (selectNodeList.isEmpty()) {
        qDebug() << "Select Drone(s) First!";
        return;
    }

    for (int nodeId: selectNodeList) {
        QVector3D controlValue = QVector3D(0.0, 0.0, ui->controlSpinBox->value());

        sendCommand(nodeId, controlValue, ui->headingSpinBox->value());
        addHistory(QString("UP to %1(m)").arg(ui->controlSpinBox->value()), nodeId);
    }
}


void CControlDialog::on_downButton_clicked() {
    if (selectNodeList.isEmpty()) {
        qDebug() << "Select Drone(s) First!";
        return;
    }

    for (int nodeId: selectNodeList) {
        QVector3D controlValue = QVector3D(0.0, 0.0, -ui->controlSpinBox->value());

        sendCommand(nodeId, controlValue, ui->headingSpinBox->value());
        addHistory(QString("DOWN to %1(m)").arg(ui->controlSpinBox->value()), nodeId);
    }
}


void CControlDialog::on_minusYButton_clicked() {
    if (selectNodeList.isEmpty()) {
        qDebug() << "Select Drone(s) First!";
        return;
    }

    for (int nodeId: selectNodeList) {
        QVector3D controlValue = QVector3D(0.0, -ui->controlSpinBox->value(), 0.0);

        sendCommand(nodeId, controlValue, ui->headingSpinBox->value());
            addHistory(QString("MOVE -y %1(m)").arg(ui->controlSpinBox->value()), nodeId);
    }
}


void CControlDialog::on_plusYButton_clicked() {
    if (selectNodeList.isEmpty()) {
        qDebug() << "Select Drone(s) First!";
        return;
    }

    for (int nodeId: selectNodeList) {
        QVector3D controlValue = QVector3D(0.0, ui->controlSpinBox->value(), 0.0);

        sendCommand(nodeId, controlValue, ui->headingSpinBox->value());
            addHistory(QString("MOVE +y %1(m)").arg(ui->controlSpinBox->value()), nodeId);
    }
}


void CControlDialog::on_minusXButton_clicked() {
    if (selectNodeList.isEmpty()) {
        qDebug() << "Select Drone(s) First!";
        return;
    }

    for (int nodeId: selectNodeList) {
        QVector3D controlValue = QVector3D(-ui->controlSpinBox->value(), 0.0, 0.0);

        sendCommand(nodeId, controlValue, ui->headingSpinBox->value());
            addHistory(QString("MOVE -x %1(m)").arg(ui->controlSpinBox->value()), nodeId);
    }
}


void CControlDialog::on_plusXButton_clicked() {
    if (selectNodeList.isEmpty()) {
        qDebug() << "Select Drone(s) First!";
        return;
    }

    for (int nodeId: selectNodeList) {
        QVector3D controlValue = QVector3D(ui->controlSpinBox->value(), 0.0, 0.0);

        sendCommand(nodeId, controlValue, ui->headingSpinBox->value());
        addHistory(QString("MOVE +x %1(m)").arg(ui->controlSpinBox->value()), nodeId);
    }
}

void CControlDialog::on_pushButton_clicked() {
    if (selectNodeList.isEmpty()) {
        qDebug() << "Select Drone(s) First!";
        return;
    }

    for (int nodeId: selectNodeList) {
        QVector3D controlValue = QVector3D(ui->movetoX->value(), ui->movetoY->value(), ui->movetoZ->value());

        sendCommand(nodeId, controlValue, ui->headingSpinBox->value());
        addHistory(QString("MOVE to %1, %2, %3").arg(controlValue.x()).arg(controlValue.y()).arg(controlValue.z()),
                   nodeId);
    }
}

void CControlDialog::sendCommand(int nodeId, QVector3D controlValue, double heading) {
    QString currentControlType = ui->type_comboBox->currentText();

    if (currentControlType == "Control") {
        double curLat = mManager->agent(nodeId)->data("GLOBAL_LAT").toDouble();
        double curLon = mManager->agent(nodeId)->data("GLOBAL_LON").toDouble();
        double curAlt = mManager->agent(nodeId)->data("GLOBAL_ALT").toDouble();
        QVector3D refLLH = mManager->agent(nodeId)->data("REF_LLH").value<QVector3D>();

        QGeoCoordinate curQGeo(curLat, curLon, curAlt);
        QGeoCoordinate refQGeo(refLLH.x(), refLLH.y(), refLLH.z());

        QVector3D newPosQVector = LLH2NED(curQGeo, refQGeo);

        newPosQVector.setX(newPosQVector.x() + controlValue.x());
        newPosQVector.setY(newPosQVector.y() + controlValue.y());

        QGeoCoordinate newPositionGeo = NED2LLH(newPosQVector, refQGeo);

        if (newPositionGeo.isValid()) {
            mManager->agent(nodeId)->cmd("MOVE", newPositionGeo.latitude(), newPositionGeo.longitude(),
                                         newPositionGeo.altitude() + controlValue.z(), heading);
        } else {
            addHistory(QString("Failed to move: Invalid newPositionGeo coordinates"), nodeId);
        }

    } else if (currentControlType == "Offboard") {
        QVector3D lpos = mManager->agent(nodeId)->data("POS").value<QVector3D>();
        QVector3D sp = QVector3D(lpos.x() + controlValue.x(),
                                 lpos.y() + controlValue.y(),
                                 lpos.z() + controlValue.z());

        mManager->agent(nodeId)->cmd("SETPOINT", sp, heading);
    }
}

void CControlDialog::sendOCMInterval() {
    if (selectNodeList.isEmpty()) {
        qDebug() << "Select Drone(s) First!";
        return;
    }

    for (int nodeId: selectNodeList) {
        mManager->agent(nodeId)->cmd("OCM_POS");
    }
}
