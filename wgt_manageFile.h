#ifndef WGT_MANAGEFILE_H
#define WGT_MANAGEFILE_H

#include <QMainWindow>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonValue>
#include <QDirIterator>
#include "qlabel.h"
#include "qpushbutton.h"
#include "qbuttongroup.h"
#include "qscrollarea.h"
#include "QGridLayout"
#include "QVBoxLayout"
#include "functionFile.h"
#include "functionAlgorithm.h"

#include "qdebug.h"
/*
 * 该子界面用于显示路径库文件，选中时发送到主界面和控制器（待实现）
*/
class wgt_manageFile : public QMainWindow
{
    Q_OBJECT
public:
    explicit wgt_manageFile(QWidget *parent = nullptr);
    ~wgt_manageFile();
signals:
    void sendInfo_fromwgtfile(QString);//报告信息发送
    void sendSeqPoseJoint(Seq_PoseJoint* );//发送路径指针用以规划


private:  
    QPushButton* btn_run;
    QPushButton* btn_hide;
    QScrollArea* scrollarea_lib;
    QWidget* subwgt_scrollarea;
    QVBoxLayout* lay_lib;
    QButtonGroup* btngroup_lib;

    Lib_File* lib_test;//数据库（惟一指定
    Seq_PoseEnd* seq_test;//点位序列（这里唯一指定是保存示教时写入的信息，写完后回收）


    void initWgt();//界面布局
    void clearBtnGroup();//清空按钮布局：每次文件更新时需要先重置
    void readFilefromLocal();//读取本地文件到lib_test

public:
    void addPoseEndtothis(double _pd,double _px,double _py,double _pz,double _pth);//接受主界面的信号，将其保存到seq_test
    void workSLOT();//关键函数
    void writeFiletoLocal(QString _name,int _att);//将seq_test保存成本地的新路径，并清空以待下一次使用
};

#endif // WGT_FILE_H
