#ifndef SENDSCDIALOG_H
#define SENDSCDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QSignalMapper>
#include <QMap>
#include <QQueue>
#include <QComboBox>
#include <QProgressBar>
#include <filemanager.h>

class CManager;

namespace Ui {
class SendSCDialog;
}

class SendSCDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SendSCDialog(CManager* aManager, QWidget *parent = 0);
    ~SendSCDialog();
    static void ftpResult(int aId, FileManager::OperationState operationState, bool isSuccess);
    static void ftpResultList(int aId, const QString& entry);
    static void ftpResultWrite(int aId, FileManager::OperationState currentOperation, bool isSuccess, uint32_t size1, uint32_t size2);

private:
    void showEvent(QShowEvent *event);
    void closeEvent(QCloseEvent *event);
    void hideEvent(QHideEvent *event);

private:
    void initNodeLayout();
    void changeButtonColor(int agentId, QColor color);
    void addNextAgentToQueue(int agentId);

private Q_SLOTS:
    void onNodeClicked(int aId);
    void updateStatus();

    void on_selectFolder_pushButton_clicked();

    void on_sendSC_pushButton_clicked();

private:
    Ui::SendSCDialog *ui;

    const int COLUMN_NODE = 10;
    const int MAX_PARALLEL_SEND = 5;
//    const int MAX_RETRY_COUNT = 5;

    CManager*           mManager;
    int                 maxId;
    QPushButton**       mNodeButton;
    QProgressBar**      mSendSCAgentProgressBar;
    QPushButton*        mSendSCButton;
    QLabel**            mNodeLabel;
    QSignalMapper*      mSignalMapper;
    QTimer				mTimer;
    QMap<int, int>      mButtonIndexMap;       // <node id, button index>
    int                 mNumAgent;
    int                 currentProgressCount = 0;
    bool                isFolderSelected = false;
    bool                isSendingSC = false;
    QQueue<int>         queueToSend;
};

#endif // SENDSCDIALOG_H
