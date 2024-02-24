#include "sendscdialog.h"
#include "ui_sendscdialog.h"
#include "manager.h"
#include "agent.h"

#include <QtMath>
#include <QApplication>
#include <filemanager.h>
#include <QMessageBox>
#include <QFileDialog>

static QList<QStringList>  mDirFiles;
static int**               currentStatus;
static int**               retryCount;
static int**               currentAgentProgress;
static int**               nodeFileSizes;

static const QString BASE_DIR = "/fs/microsd";
static const QString SC_DIR = "/sc";
static QString SPECIFIC_SC_DIR = "";

static const int STATUS_INITIAL = 0;
static const int STATUS_CHECK_SC_DIR = 1;
static const int STATUS_CHECK_SPECIFIC_SC_DIR = 10;
static const int STATUS_CHECK_NODE_FILE = 20;
static const int STATUS_REMOVE_NODE_FILE = 30;
static const int STATUS_TRANSFER = 40;
static const int STATUS_CREATE_SC_DIR = 100;
static const int STATUS_CREATE_SPECIFIC_SC_DIR = 110;
static const int STATUS_FAIL = 999;

static const int STATUS_PROGRESS = 1;
static const int STATUS_DONE = 2;

SendSCDialog::SendSCDialog(CManager* aManager, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SendSCDialog)
{
    ui->setupUi(this);

    mNumAgent = 0;
    mNodeButton =NULL;
    mNodeLabel = NULL;
    mSignalMapper = new QSignalMapper (this) ;

    mManager = aManager;

    mSendSCButton = ui->sendSC_pushButton;
}

void SendSCDialog::initNodeLayout()
{
    mNumAgent = mManager->numOfAgent();

    // free allocated memory
    if ( mNodeButton != NULL ) delete mNodeButton;
    if ( mNodeLabel != NULL ) delete mNodeLabel;
    disconnect(mSignalMapper, 0, 0, 0);


    // create button and label
    mNodeButton = new QPushButton*[mNumAgent];
    mNodeLabel = new QLabel*[mNumAgent];
    mSendSCAgentProgressBar = new QProgressBar*[mNumAgent];

    int i = 0;
    QMap<int, IAgent*> agentsMap = mManager->agents();

    // get max id number
    QMap<int, IAgent*>::iterator agentIterator;
    for (agentIterator = agentsMap.begin(); agentIterator != agentsMap.end(); ++agentIterator){
//    foreach (IAgent* agent, agentsMap) {
        int id = agentIterator.value()->id();
        if ( id > maxId ) {
            maxId = id;
        }
    }

    currentStatus = new int*[maxId + 1];
    retryCount = new int*[maxId + 1];
    currentAgentProgress = new int*[maxId + 1];
    nodeFileSizes = new int*[maxId + 1];

    for(i=0;i<=maxId;i++) {
        currentStatus[i] = new int;
        *currentStatus[i] = 0;

        retryCount[i] = new int;
        *retryCount[i] = 0;

        currentAgentProgress[i] = new int;
        *currentAgentProgress[i] = 0;

        nodeFileSizes[i] = new int;
        *nodeFileSizes[i] = 0;
    }

    // create agent button
    i = 0;
//    foreach (IAgent* agent, agentsMap) {
    for (agentIterator = agentsMap.begin(); agentIterator != agentsMap.end(); ++agentIterator){
        int id = agentIterator.value()->id();
        int sysid = agentIterator.value()->data("SYSID").toInt();

        QStringList qstringList;
        mDirFiles.append(qstringList);

        // create button
        mNodeButton[i] = new QPushButton(this);
        mNodeButton[i]->setFixedSize(70,40);
        mNodeButton[i]->setStyleSheet("font-size: 17px;font: bold");
        //mNodeButton[i]->setText("--");
        QString name = QString("X%1\n(%2)").arg(sysid).arg(id);
        mNodeButton[i]->setText(name);

        // create label
        mNodeLabel[i] = new QLabel(this);
        mNodeLabel[i]->setFixedSize(90,20);
        QFont f("Arial", 10);
        mNodeLabel[i]->setFont(f);
        mNodeLabel[i]->setWordWrap(true);
        mNodeLabel[i]->setText(" ");
        mNodeLabel[i]->setAlignment(Qt::AlignCenter);

        // create progressbar
        mSendSCAgentProgressBar[i] = new QProgressBar(this);
        mSendSCAgentProgressBar[i]->setFixedSize(70,20);
        mSendSCAgentProgressBar[i]->setValue(0);

        // add widget to layout
        int tableMaxNum = (qCeil((float)maxId/COLUMN_NODE))*COLUMN_NODE;
        int row = (tableMaxNum-id)/COLUMN_NODE;
        int col = (id-1)%COLUMN_NODE;
        ui->node_gridLayout->addWidget(mSendSCAgentProgressBar[i], row*3+0, col,1,1);
        ui->node_gridLayout->addWidget(mNodeButton[i],row*3+1, col,1,1);
        ui->node_gridLayout->addWidget(mNodeLabel[i], row*3+2, col,1,1);

        // set alignment
        ui->node_gridLayout->setAlignment(mSendSCAgentProgressBar[i], Qt::AlignHCenter);
        ui->node_gridLayout->setAlignment(mNodeButton[i], Qt::AlignHCenter);
        ui->node_gridLayout->setAlignment(mNodeLabel[i], Qt::AlignHCenter);


        // Mapping
        mButtonIndexMap[id] = i;

        // connection
        connect (mNodeButton[i], SIGNAL(clicked(bool)), mSignalMapper, SLOT(map())) ;
        mSignalMapper->setMapping (mNodeButton[i], id);

        i++;
    }

//    isFolderSelected = true;        // SHOULD BE MODIFIED.
    isFolderSelected = false;
//    ui->selectedFolder_Label->setText("/home/dongoo/test_py/korea2");       // SHOULD BE MODIFIED.
    ui->selectedFolder_Label->setText("None");

    connect (mSignalMapper, SIGNAL(mapped(int)), this, SLOT(onNodeClicked(int))) ;

    connect(&mTimer, SIGNAL(timeout()), this, SLOT(updateStatus()));
    mTimer.setInterval(1000);
}

SendSCDialog::~SendSCDialog()
{
    if ( mNodeButton != NULL ) delete mNodeButton;
    if ( mNodeLabel != NULL ) delete mNodeLabel;
    if ( mSignalMapper != NULL ) delete mSignalMapper;
    mDirFiles.clear();
    delete currentStatus;
    delete retryCount;
    delete currentAgentProgress;
    delete nodeFileSizes;

    delete ui;
}

void SendSCDialog::showEvent(QShowEvent *event)
{
    Q_UNUSED(event);

    this->move(500,200);
    if ( mNumAgent == 0 ) {
        initNodeLayout();
    }
    mTimer.start();
}

void SendSCDialog::closeEvent(QCloseEvent *event)
{
    Q_UNUSED(event);
    mTimer.stop();
}

void SendSCDialog::hideEvent(QHideEvent *event)
{
    Q_UNUSED(event);
    mTimer.stop();
}

void SendSCDialog::onNodeClicked(int aId)
{
    if (isFolderSelected) {
        queueToSend.enqueue(aId);
    } else {
        QMessageBox msgBox;
        msgBox.setText("Please open folder first");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
}

void SendSCDialog::ftpResultWrite(int aId, FileManager::OperationState currentOperation, bool isSuccess, uint32_t cur_size, uint32_t total_size) {
//    qDebug("ftpResultWrite, aId : %d, size1 : %d, size2 : %d", aId, cur_size, total_size);
    if (isSuccess && currentAgentProgress != NULL) {
        *currentAgentProgress[aId] = int((double)cur_size/total_size*100);
//        qDebug("currentAgentProgress : %d", *currentAgentProgress[aId]);
    } else {
        qDebug("ftpResultWrite, aId : %d, size1 : %d, size2 : %d", aId, cur_size, total_size);
    }
}

void SendSCDialog::ftpResult(int aId, FileManager::OperationState currentOperation, bool isSuccess) {
    qDebug("SendSCDialog ftpResponse -> sysId : %d, operationsState : %d, isSuccess : %d",aId, currentOperation, isSuccess);
    if (mDirFiles.size() > 0) {
        if (currentOperation == 2 && isSuccess){ // kCOList
            // LIST DIR SUCCESS
            if (*currentStatus[aId] == STATUS_CHECK_SC_DIR + STATUS_PROGRESS) {
                QStringList list = mDirFiles.at(aId - 1);
                for(int i=0; i< list.size(); i++) {
                    QString item = list.at(i);
//                    qDebug("STATUS_CHECK_SC_DIR + STATUS_PROGRESS => %s",item.toLatin1().data());
                    if (item.compare("sc", Qt::CaseSensitive) == 0) {
                        *currentStatus[aId] = STATUS_CHECK_SC_DIR + STATUS_DONE;
                        break;
                    }
                }

                if (*currentStatus[aId] == STATUS_CHECK_SC_DIR + STATUS_PROGRESS) {       // There is no sc directory
                    // CREATE_DIR
//                    qDebug("There is no sc directory. Create it!");
                    *currentStatus[aId] = STATUS_CREATE_SC_DIR;
                }
            } else if (*currentStatus[aId] == STATUS_CHECK_SPECIFIC_SC_DIR + STATUS_PROGRESS) {
                QStringList list = mDirFiles.at(aId - 1);
                for(int i=0; i< list.size(); i++) {
                    QString item = list.at(i);
                    if (item.compare(SPECIFIC_SC_DIR.mid(1,SPECIFIC_SC_DIR.size()-1), Qt::CaseSensitive) == 0) {
                        *currentStatus[aId] = STATUS_CREATE_SPECIFIC_SC_DIR + STATUS_DONE;
                        break;
                    }
                }

                if (*currentStatus[aId] == STATUS_CHECK_SPECIFIC_SC_DIR + STATUS_PROGRESS) {
                    *currentStatus[aId] = STATUS_CREATE_SPECIFIC_SC_DIR;
                }
            } else if (*currentStatus[aId] == STATUS_CHECK_NODE_FILE + STATUS_PROGRESS) {
                QStringList list = mDirFiles.at(aId - 1);
                for(int i=0; i< list.size(); i++) {
                    QStringList items = list.at(i).split("\t");
//                    qDebug("CHECK_NODE_FILE %d : %s, items.size : %d, items.at(0) : %s", i, list.at(i).toLatin1().data(), items.size(), items.at(0).toLatin1().data());
                    if (items.size() > 1 && items.at(0).compare("node_"+QString::number(aId)+".txt", Qt::CaseSensitive) == 0) {
                        // CHECK SIZE
//                        qDebug("CHECK SIZE. on agent : %d, local : %d", items.at(1).toInt(), *nodeFileSizes[aId]);
                        if (items.at(1).toInt() != *nodeFileSizes[aId]) {
                            // Remove File
                            *currentStatus[aId] = STATUS_REMOVE_NODE_FILE;
                        } else {
                            *currentStatus[aId] = STATUS_TRANSFER + STATUS_DONE;
                        }
                        break;
                    }
                }

                if (*currentStatus[aId] == STATUS_CHECK_NODE_FILE + STATUS_PROGRESS) {
                    // No node_N.txt file
//                    qDebug("No node file! Start upload");
                    *currentStatus[aId] = STATUS_TRANSFER;
                }
            }
        } else if (currentOperation == 9 && isSuccess) {    // kCOCreateDir
            // CREATE DIR SUCCESS
            if (*currentStatus[aId] == STATUS_CREATE_SC_DIR + STATUS_PROGRESS) {
                *currentStatus[aId] = STATUS_CREATE_SC_DIR + STATUS_DONE;
            } else if (*currentStatus[aId] == STATUS_CREATE_SPECIFIC_SC_DIR + STATUS_PROGRESS) {
                *currentStatus[aId] = STATUS_CREATE_SPECIFIC_SC_DIR + STATUS_DONE;
            }
        } else if (currentOperation == 11 && isSuccess) {
            if (*currentStatus[aId] == STATUS_REMOVE_NODE_FILE + STATUS_PROGRESS) {
//                qDebug("Start upload! after remove node file.");
                *currentStatus[aId] = STATUS_TRANSFER;
            }
        } else if (currentOperation == 12 && isSuccess) {
            if (*currentStatus[aId] == STATUS_TRANSFER + STATUS_PROGRESS) {
//                qDebug("Upload Complete!");
                *currentStatus[aId] = STATUS_TRANSFER + STATUS_DONE;
            }
        } else if (!isSuccess) {
            *currentStatus[aId] = STATUS_FAIL;
        }
    }
}

void SendSCDialog::ftpResultList(int aId, const QString& entry) {
    if (mDirFiles.size() > 0) {
        QStringList list = mDirFiles.at(aId - 1);
        list.append(entry.mid(1,entry.count() - 1));
        mDirFiles.replace(aId-1, list);
    } else {
        qDebug("sysId : %d, entry count: %d, %s",aId, entry.count(), entry.toLatin1().data());
    }
}

void SendSCDialog::updateStatus()
{
    QString normalStyle = "font-size:30px;font:bold;";
    QString readyToFlyStyle = "font-size: 30px;font: bold;background-color:rgb(0,150,0);";
    QString emergencyStyle = normalStyle + QString("background-color: rgb(200,0,0);");

    if (isSendingSC) {
        int queueSize = queueToSend.size();
        for(int i = 0; i<queueSize; i++) {
            int agentId = queueToSend.dequeue();
            *currentStatus[agentId] = STATUS_CHECK_SC_DIR + STATUS_PROGRESS;
            mNodeLabel[i]->setText("Check sc..");
            QStringList emptyList;
            mDirFiles.replace(agentId-1, emptyList);
            mManager->agent(agentId)->cmd("LIST_DIR",BASE_DIR);
            changeButtonColor(agentId, Qt::yellow);
        }
    }

    QMap<int, IAgent*> agentsMap = mManager->agents();
    QMap<int, IAgent*>::iterator i;
    for (i = agentsMap.begin(); i != agentsMap.end(); ++i){
//    foreach (IAgent* agent, agentsMap) {
        int id = i.value()->id();
        int sysid = i.value()->data("SYSID").toInt();
        int index = mButtonIndexMap[id];

        QString name = QString("X%1\n(%2)").arg(sysid).arg(id);
        mNodeButton[index]->setText(name);

        QStringList emptyList;
        if (*currentStatus[id] == STATUS_CREATE_SC_DIR) {
            *currentStatus[id] = STATUS_CREATE_SC_DIR + STATUS_PROGRESS;
            mNodeLabel[index]->setText("Create sc..");
            mManager->agent(id)->cmd("CREATE_DIR", BASE_DIR + SC_DIR);
//            qDebug("CREATE DIR! SC_DIR");
        } else if (*currentStatus[id] == STATUS_CREATE_SC_DIR + STATUS_DONE) {
            *currentStatus[id] = STATUS_CHECK_SC_DIR + STATUS_PROGRESS;
            mNodeLabel[index]->setText("Check sc..");
            mDirFiles.replace(id-1, emptyList);
            mManager->agent(id)->cmd("LIST_DIR", BASE_DIR);
//            qDebug("LIST DIR! BASE_DIR");
        } else if (*currentStatus[id] == STATUS_CHECK_SC_DIR + STATUS_DONE) {
            *currentStatus[id] = STATUS_CHECK_SPECIFIC_SC_DIR + STATUS_PROGRESS;
            mNodeLabel[index]->setText("Check S sc..");
            mDirFiles.replace(id-1, emptyList);
            mManager->agent(id)->cmd("LIST_DIR", BASE_DIR + SC_DIR);
//            qDebug("LIST DIR! SC_DIR");
        } else if (*currentStatus[id] == STATUS_CREATE_SPECIFIC_SC_DIR) {
            *currentStatus[id] = STATUS_CREATE_SPECIFIC_SC_DIR + STATUS_PROGRESS;
            mNodeLabel[index]->setText("Create S sc..");
            mManager->agent(id)->cmd("CREATE_DIR", BASE_DIR + SC_DIR + SPECIFIC_SC_DIR);
//            qDebug("CREATE DIR! SPECIFIC_SC_DIR");
        } else if (*currentStatus[id] == STATUS_CREATE_SPECIFIC_SC_DIR + STATUS_DONE) {
            *currentStatus[id] = STATUS_CHECK_NODE_FILE + STATUS_PROGRESS;
            mNodeLabel[index]->setText("Check node file..");
            mDirFiles.replace(id-1, emptyList);
            mManager->agent(id)->cmd("LIST_DIR", BASE_DIR + SC_DIR + SPECIFIC_SC_DIR);
//            qDebug("LIST DIR! SPECIFIC_SC_DIR");
        } else if (*currentStatus[id] == STATUS_REMOVE_NODE_FILE) {
            *currentStatus[id] = STATUS_REMOVE_NODE_FILE + STATUS_PROGRESS;
            mNodeLabel[index]->setText("Remove node file..");
            mManager->agent(id)->cmd("REMOVE_SCEN", BASE_DIR + SC_DIR + SPECIFIC_SC_DIR + "/node_" + QString::number(id) + ".txt");
//            qDebug("REMOVE NODE FILE!");
        } else if (*currentStatus[id] == STATUS_TRANSFER) {
            *currentStatus[id] = STATUS_TRANSFER + STATUS_PROGRESS;
            mNodeLabel[index]->setText("Uploading..");
            mManager->agent(id)->cmd("UPLOAD_SCEN", BASE_DIR + SC_DIR + SPECIFIC_SC_DIR, ui->selectedFolder_Label->text() + "/node_" + QString::number(id) + ".txt");
        } else if (*currentStatus[id] == STATUS_TRANSFER + STATUS_PROGRESS) {
            mSendSCAgentProgressBar[index]->setValue(*currentAgentProgress[id]);
        } else if (*currentStatus[id] == STATUS_TRANSFER + STATUS_DONE) {
            mNodeLabel[id-1]->setText(" ");
            mSendSCAgentProgressBar[index]->setValue(100);
            changeButtonColor(id, Qt::green);
            if (isSendingSC) {
                addNextAgentToQueue(id);
            }
        } else if (*currentStatus[id] == STATUS_FAIL) {
            changeButtonColor(id, Qt::red);
        }
    }
}

void SendSCDialog::addNextAgentToQueue(int agentId) {
    int nextAgentId = agentId + 1;
    while(nextAgentId <= maxId && !mManager->hasAgent(nextAgentId) && *currentStatus[nextAgentId] == STATUS_INITIAL) {
        nextAgentId++;
    }

    if (nextAgentId <= maxId) {
        queueToSend.enqueue(nextAgentId);
    }
}

void SendSCDialog::changeButtonColor(int agentId, QColor color){
    int buttonIndex = mButtonIndexMap[agentId];
    QPalette pal = mNodeButton[buttonIndex]->palette();
    pal.setColor(QPalette::Button, color);
    mNodeButton[buttonIndex]->setAutoFillBackground(true);
    mNodeButton[buttonIndex]->setPalette(pal);
    mNodeButton[buttonIndex]->update();
}

void SendSCDialog::on_selectFolder_pushButton_clicked()
{
    const QMap<int, IAgent*> agents = mManager->agents();
    if (agents.size() < 1) {
        QMessageBox msgBox;
        msgBox.setText("Open conf file first!");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    } else {
        QString folderName = QFileDialog::getExistingDirectory(
                    this,
                    tr("Open Scenario Folder"),
                    QString(CONFIG_FILE_PATH),
                    QFileDialog::ShowDirsOnly);

        if ( !folderName.isEmpty() ) {
            // Check node files are exist in selected directory.
            bool fileExist = true;
            QString absentFile = "";
            for(int i=1;i<=agents.size();i++) {
                QString filePath = folderName + "/node_" + QString::number(i) + ".txt";
                QFileInfo fileName(filePath);
                if (!(fileName.exists() && fileName.isFile())) {
                    absentFile = "node_" + QString::number(i) + ".txt";
                    fileExist = false;
                    break;
                }
                *nodeFileSizes[i] = fileName.size();
            }

            if(fileExist) {
                SPECIFIC_SC_DIR = folderName.mid(folderName.lastIndexOf("/"), folderName.size()-1);
                isFolderSelected = true;
                ui->selectedFolder_Label->setText(folderName);
            } else {
                QMessageBox msgBox;
                msgBox.setText("Some files are missing : " + absentFile);
                msgBox.setStandardButtons(QMessageBox::Ok);
                msgBox.exec();
            }
        }
    }
}

void SendSCDialog::on_sendSC_pushButton_clicked()
{
    if (isFolderSelected) {
        if (!isSendingSC) {
            mSendSCButton->setText("Stop Sending");
            isSendingSC = !isSendingSC;
            int addQueueCount = 0;
            QMap<int, IAgent*> agentsMap = mManager->agents();
            QMap<int, IAgent*>::iterator i;
            for (i = agentsMap.begin(); i != agentsMap.end(); ++i){
//            foreach (IAgent* agent, agentsMap) {
                int id = i.value()->id();
                if (addQueueCount < MAX_PARALLEL_SEND) {
                    queueToSend.enqueue(id);
                    addQueueCount++;
                }
            }
        } else {
            mSendSCButton->setText("Send Scenario");
            isSendingSC = !isSendingSC;
        }
    } else {
        QMessageBox msgBox;
        msgBox.setText("Please open folder first");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
}
