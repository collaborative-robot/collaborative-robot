#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qdebug.h"
#include <QTimer>
#include <ctime>
#include "qradiobutton.h"
#include "qslider.h"
#include "qlineedit.h"
#include "qlabel.h"
#include "qpushbutton.h"
#include "glwidget.h"
#include "wgt_manageFile.h"
#include "qgroupbox.h"
#include "qtextbrowser.h"

#include "qchartview.h"
#include "qvalueaxis.h"
#include "qlineseries.h"
using namespace QtCharts;
/*
 * 母窗口
 * 校对完毕；鑫2024.6.13
*/
namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    //总的刷新函数，调用scara的go类
    void go();
    //摄像机参数，拖动调整
    void diy_verticalSliderPitch_valueChanged(int value);
    void diy_horizontalSliderYaw_valueChanged(int value);
    //示教模式下的关节角度，拖动调整
    void diy_horizontalSliderq1_valueChanged(int value);
    void diy_horizontalSliderq2_valueChanged(int value);
    void diy_horizontalSliderq3_valueChanged(int value);
    void diy_horizontalSliderq4_valueChanged(int value);
    void diy_horizontalSliderq5_valueChanged(int value);
    //重新设置目标模式的末端位姿
    void diy_commandLinkButtonGo_clicked();
    //更换运行模式
    void diy_radioButtonSimple_clicked();
    void diy_radioButtonTarget_clicked();


    //界面初始化
    void initWgt();

private:
    Ui::MainWindow *ui;

    //时间
    QTimer tmr;
    double t, dt;//t：保存经过的总时间，dt：每次刷新的时间间隔
    clock_t clk;//时间类，获得绝对时间，用以计算时间间隔dt

    //算法执行
    size_t num_spline=0;//执行到序列中的第几个
    Seq_PoseJoint* seq_jointforwork;//待执行的序列

    //主界面
    QRadioButton* rbtn_simple;
    QRadioButton* rbtn_target;
    QRadioButton* rbtn_work;
    QGroupBox* gbox_joint;
    QGroupBox* gbox_target;
    QSlider* slider_q1;
    QSlider* slider_q2;
    QSlider* slider_q3;
    QSlider* slider_q4;
    QSlider* slider_q5;
    QSlider* slider_pitch;
    QSlider* slider_yaw;
    QLineEdit* edit_pd;
    QLineEdit* edit_px;
    QLineEdit* edit_py;
    QLineEdit* edit_pz;
    QLineEdit* edit_pth; 
    QLineEdit* edit_name;
    QLineEdit* edit_attri;
    QLabel* lb_q1;
    QLabel* lb_q2;
    QLabel* lb_q3;
    QLabel* lb_q4;
    QLabel* lb_q5;
    QLabel* lb_pd;
    QLabel* lb_px;
    QLabel* lb_py;
    QLabel* lb_pz;
    QLabel* lb_pth;
    QPushButton* btn_settarget;
    QPushButton* btn_addposeend;
    QPushButton* btn_writeseq;
    QPushButton* btn_openlib;
    GLWidget* wgt_sim;//注意是自定义的类，不是继承对象
    QTextBrowser* txtbrow;

    //文件管理界面
    wgt_manageFile* subwgt_file;

    //曲线图界面
    QChartView *chartview;
    QChart *chart;
    QValueAxis *axis_x;
    QValueAxis *axis_y;
    QLineSeries *dataline_end;//多个曲线可类似添加,此处仅末端速度
    double d_last;//保存上一个时间的末端数据，用以计算速度
    double x_last;
    double y_last;
    double z_last;
    //double th_last;这个没用上
    double y_max;
    double y_min;//数据最大最小值，更改坐标轴范围以显示完全。

signals:
    void sendPoseEend(double _pd,double _px,double _py,double _pz,double _pth);//向文件界面发送一个末端信息，用以保存路径
    void sendInfoSeq(QString _name,int _att);//向文件界面发送末端其余参数，用以保存路径
public:
    void showStatusInfo(QString _str);//在状态栏显示信息
    void startSim(Seq_PoseJoint* seq_in);//接受插补后的路径，开始仿真

};

#endif // MAINWINDOW_H
