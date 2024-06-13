#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QHBoxLayout"
#include "QVBoxLayout"
#include <QIntValidator>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    dt(0.0)
{
    ui->setupUi(this);
    initWgt();
    robot = new Scara(this);
    clk = clock();

    connect(&tmr, &QTimer::timeout, this, &MainWindow::go);
    tmr.setInterval(5);
    tmr.start();  
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::go()
{
    dt = 1.0 * (clock() - clk) / CLOCKS_PER_SEC;
    clk = clock();
    //一：参数迭代区:获得最新的机械臂参数
    //情况1：在目标模式下：scara类自己更新一次参数：实际上在非目标模式下dq=0,即这个函数实际上不变。
    robot->go(dt);
    //情况2：算法模式下外部修改参数(算法模式下
    if(rbtn_work->isChecked()){
        robot->setq1(seq_jointforwork->pointer_pose_joint[num_spline]->q1);
        robot->setq2(seq_jointforwork->pointer_pose_joint[num_spline]->q2);
        robot->setq3(seq_jointforwork->pointer_pose_joint[num_spline]->q3);
        robot->setq4(seq_jointforwork->pointer_pose_joint[num_spline]->q4);
        robot->setq5(seq_jointforwork->pointer_pose_joint[num_spline]->q5);
        num_spline++;
        //一个算法完成时回到示教模式
        if(num_spline==seq_jointforwork->cnt_now){
            robot->setMode(SIMPLE);
            rbtn_simple->setChecked(true);
            gbox_joint->setEnabled(true);
            gbox_target->setDisabled(true);
            num_spline=0;
            txtbrow->append("执行完毕");
            freeSeqPoseJoint(seq_jointforwork);//执行完的序列释放掉！
            //后续优化：如果有多个序列等待实现：1如何存储，2如何连续实现（这个好像不难，问题在于第一个）3如何复位，即当前位姿和算法序列第一个不一致
        }
    }
    //情况3：示教模式，在ui上拖动更新,因此这里为空


    //二：显示各参数
    QString text;
    //获取各关节角度并显示。只读
    text.sprintf("%.3f", robot->getq1());
    lb_q1->setText(text);
    text.sprintf("%.3f", robot->getq2());
    lb_q2->setText(text);
    text.sprintf("%.3f", robot->getq3());
    lb_q3->setText(text);
    text.sprintf("%.3f", robot->getq4());
    lb_q4->setText(text);
    text.sprintf("%.3f", robot->getq5());
    lb_q5->setText(text);


    //获取末端并显示。只读
    text.sprintf("%.3f", robot->getDL());
    lb_pd->setText(text);
    text.sprintf("%.3f", robot->getX());
    lb_px->setText(text);
    text.sprintf("%.3f", robot->getY());
    lb_py->setText(text);
    text.sprintf("%.3f", robot->getZ());
    lb_pz->setText(text);    
    text.sprintf("%.3f", robot->getTH());
    lb_pth->setText(text);

    //额外的：非示教模式下还要更新拖动块显示
    if(!rbtn_simple->isChecked()) {
        //系统长度单位是米，对于slider控件而言比较小，故乘100后显示。（同样读取时也要除以100）
        slider_q1->setValue(100.0*robot->getq1());
        slider_q2->setValue(100.0*robot->getq2());
        slider_q3->setValue(robot->getq3());
        slider_q4->setValue(robot->getq4());
        slider_q5->setValue(robot->getq5());
    }

    //三：用模型的各参数差分计算若干观测量变化并更新图例
      //初始为0
    if(dataline_end->count()==0){
        d_last=robot->getDL();
        x_last=robot->getX();
        y_last=robot->getY();
        z_last=robot->getZ();
        y_min=-2;
        y_max=2;
        dataline_end->append(QPointF(t,0));
    }
    else{
        //如果数目过多，先去掉第一个
        if(dataline_end->count()>=1000){
            dataline_end->remove(0);
        }
        //添加新速度到序列
        double vd_temp = (robot->getDL()-d_last)/0.005;
        double vx_temp = (robot->getX()-x_last)/0.005;
        double vy_temp = (robot->getY()-y_last)/0.005;
        double vz_temp = (robot->getZ()-z_last)/0.005;//控制器间隔0,005ms,这样子算末端确实是匀速，但如果用dt，不是匀速，机器时间不是真的等间隔..
        double vend_temp = sqrt((vd_temp+vx_temp)*(vd_temp+vx_temp)+vy_temp*vy_temp+vz_temp*vz_temp);
        dataline_end->append(QPointF(t,vend_temp));
        //更新坐标轴范围
        axis_x->setMin(dataline_end->at(0).x());
        axis_x->setMax(dataline_end->at(dataline_end->count()-1).x());
        y_min=-2;y_max=2;
        for (int k=0;k<dataline_end->count() ;k++ ) {
            if(dataline_end->at(k).y()>y_max){y_max=(dataline_end->at(k).y()+1);}
            if(dataline_end->at(k).y()<y_min){y_min=(dataline_end->at(k).y()-1);}
        }
        axis_y->setMin(y_min);
        axis_y->setMax(y_max);
        d_last=robot->getDL();
        x_last=robot->getX();
        y_last=robot->getY();
        z_last=robot->getZ();
    }


    //四：根据最新参数建模
    wgt_sim->updateGL();
    //五：总时间迭代：根据末端算速度有一个相位的延迟，因此计算速度后再更新成最新的时间
    //t += dt;//这里单位为s
    t=t+0.005;
}

void MainWindow::diy_verticalSliderPitch_valueChanged(int value)
{
    wgt_sim->setPitch(value);
}
void MainWindow::diy_horizontalSliderYaw_valueChanged(int value)
{
    wgt_sim->setYaw(value);
}

void MainWindow::diy_horizontalSliderq1_valueChanged(int value)
{
    if(rbtn_simple->isChecked())robot->setq1(0.01*value);
}
void MainWindow::diy_horizontalSliderq2_valueChanged(int value)
{
    if(rbtn_simple->isChecked())robot->setq2(0.01*value);
}
void MainWindow::diy_horizontalSliderq3_valueChanged(int value)
{
    if(rbtn_simple->isChecked())robot->setq3(value);
}
void MainWindow::diy_horizontalSliderq4_valueChanged(int value)
{
    if(rbtn_simple->isChecked())robot->setq4(value);
}
void MainWindow::diy_horizontalSliderq5_valueChanged(int value)
{
    if(rbtn_simple->isChecked())robot->setq5(value);
}

void MainWindow::diy_commandLinkButtonGo_clicked()
{
    robot->setTarget(edit_pd->text().toDouble(),edit_px->text().toDouble(), edit_py->text().toDouble(), edit_pz->text().toDouble(),edit_pth->text().toDouble());
}

void MainWindow::diy_radioButtonSimple_clicked()
{
    robot->setMode(SIMPLE);

    gbox_joint->setEnabled(true);
    gbox_target->setDisabled(true);
}
void MainWindow::diy_radioButtonTarget_clicked()
{
    robot->setMode(TARGET);

    gbox_joint->setDisabled(true);
    gbox_target->setEnabled(true);
}

void MainWindow::showStatusInfo(QString _str)
{
    txtbrow->append(_str);
}

void MainWindow::initWgt()
{
    //qmainwindow没有widget，无法使用layout，必须这样处理。（或者直接改继承类）
    QWidget* wgt0 = new QWidget;
    this->setCentralWidget(wgt0);
    this->setFixedSize(QSize(1500,900));
    //左边部分1:曲线图
    chartview = new QChartView;
    chart = new QChart;
    dataline_end =new QLineSeries;
       //区域
    chart->setTitle(tr("随时间变化"));
       //曲线
    dataline_end->setName("末端速度");
    dataline_end->setColor(Qt::blue);
    dataline_end->setPen(QPen(Qt::blue,2));
    chart->addSeries(dataline_end);
       //x
    axis_x =new QValueAxis;
    axis_x->setTitleText("时间");
    axis_x->setRange(0,5);//设置值域
    chart->addAxis(axis_x,Qt::AlignBottom);//沿底边显示
    dataline_end->attachAxis(axis_x);//绑定x轴
       //y
    axis_y =new QValueAxis;
    axis_y->setTitleText("速度");
    axis_y->setRange(-2,2);//设置值域
    chart->addAxis(axis_y,Qt::AlignLeft);//沿底边显示
    dataline_end->attachAxis(axis_y);//绑定y轴
       //绑定
    chartview->setChart(chart);
    //左边部分2：信息输出栏
    txtbrow = new QTextBrowser;//输出显示栏
    //左边部分3：路径信息输入栏
    edit_name = new QLineEdit;edit_name->setText("undefied");//路径名
    edit_attri = new QLineEdit;edit_attri->setValidator(new QIntValidator(1,1000,this));edit_attri->setText("0");//路径属性
    QVBoxLayout* lay_left = new QVBoxLayout;
    lay_left->addWidget(chartview,3);
    lay_left->addWidget(txtbrow,1);
    lay_left->addWidget(edit_name,1);
    lay_left->addWidget(edit_attri,1);
    //中间部分1：GL界面
    wgt_sim = new GLWidget;
    slider_pitch = new QSlider;slider_pitch->setOrientation(Qt::Vertical);
    slider_pitch->setMinimum(-90);slider_pitch->setMaximum(90);slider_pitch->setValue(30);
    QHBoxLayout* lay_mid_1 = new QHBoxLayout;
    lay_mid_1->addWidget(slider_pitch,1);
    lay_mid_1->addWidget(wgt_sim,20);
    //中间部分2
    slider_yaw = new QSlider;slider_yaw->setOrientation(Qt::Horizontal);
    slider_yaw->setMinimum(-180);slider_yaw->setMaximum(180);slider_yaw->setValue(0);
    //中间部分3：显示关节角度和末端位姿
    QLabel* lb_showjoint1 = new QLabel;lb_showjoint1->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);lb_showjoint1->setText("关节1转角/度");//lb_showjoint1->setAlignment(Qt::AlignCenter);
    QLabel* lb_showjoint2 = new QLabel;lb_showjoint2->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);lb_showjoint2->setText("关节2转角/度");//lb_showjoint2->setAlignment(Qt::AlignCenter);
    QLabel* lb_showjoint3 = new QLabel;lb_showjoint3->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);lb_showjoint3->setText("关节3转角/度");//lb_showjoint3->setAlignment(Qt::AlignCenter);
    QLabel* lb_showjoint4 = new QLabel;lb_showjoint4->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);lb_showjoint4->setText("关节4转角/度");//lb_showjoint4->setAlignment(Qt::AlignCenter);
    QLabel* lb_showjoint5 = new QLabel;lb_showjoint5->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);lb_showjoint5->setText("关节5转角/度)");//lb_showjoint5->setAlignment(Qt::AlignCenter);
    lb_q1 = new QLabel;lb_q1->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    lb_q2 = new QLabel;lb_q2->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    lb_q3 = new QLabel;lb_q3->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    lb_q4 = new QLabel;lb_q4->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    lb_q5 = new QLabel;lb_q5->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    QLabel* lb_showpd = new QLabel;lb_showpd->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);lb_showpd->setText("侧位移/米");
    QLabel* lb_showpx = new QLabel;lb_showpx->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);lb_showpx->setText("x位移/米");
    QLabel* lb_showpy = new QLabel;lb_showpy->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);lb_showpy->setText("y位移/米");
    QLabel* lb_showpz = new QLabel;lb_showpz->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);lb_showpz->setText("z位移/米");
    QLabel* lb_showpth = new QLabel;lb_showpth->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);lb_showpth->setText("末端转角/度");
    lb_pd = new QLabel;lb_pd->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    lb_px = new QLabel;lb_px->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    lb_py = new QLabel;lb_py->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    lb_pz = new QLabel;lb_pz->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    lb_pth = new QLabel;lb_pth->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);  
    QGridLayout* lay_mid_2 = new QGridLayout;
    lay_mid_2->addWidget(lb_showjoint1,0,0);
    lay_mid_2->addWidget(lb_showjoint2,0,1);
    lay_mid_2->addWidget(lb_showjoint3,0,2);
    lay_mid_2->addWidget(lb_showjoint4,0,3);
    lay_mid_2->addWidget(lb_showjoint5,0,4);
    lay_mid_2->addWidget(lb_q1,1,0);
    lay_mid_2->addWidget(lb_q2,1,1);
    lay_mid_2->addWidget(lb_q3,1,2);
    lay_mid_2->addWidget(lb_q4,1,3);
    lay_mid_2->addWidget(lb_q5,1,4);
    lay_mid_2->addWidget(lb_showpd,2,0);
    lay_mid_2->addWidget(lb_showpx,2,1);
    lay_mid_2->addWidget(lb_showpy,2,2);
    lay_mid_2->addWidget(lb_showpz,2,3);
    lay_mid_2->addWidget(lb_showpth,2,4);
    lay_mid_2->addWidget(lb_pd,3,0);
    lay_mid_2->addWidget(lb_px,3,1);
    lay_mid_2->addWidget(lb_py,3,2);
    lay_mid_2->addWidget(lb_pz,3,3);
    lay_mid_2->addWidget(lb_pth,3,4);
    //中间部分4
    btn_addposeend = new QPushButton;btn_addposeend->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);btn_addposeend->setText("添加点位");
    btn_writeseq = new QPushButton;btn_writeseq->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);btn_writeseq->setText("保存路径");
    btn_openlib = new QPushButton;btn_openlib->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);btn_openlib->setText("打开数据库");
    QHBoxLayout* lay_mid_3 = new QHBoxLayout;
    lay_mid_3->addWidget(btn_addposeend,1);
    lay_mid_3->addWidget(btn_writeseq,1);
    lay_mid_3->addWidget(btn_openlib,1);
    QVBoxLayout* lay_mid = new QVBoxLayout;
    lay_mid->addLayout(lay_mid_1,8);
    lay_mid->addWidget(slider_yaw,1);
    lay_mid->addLayout(lay_mid_2,2);
    lay_mid->addLayout(lay_mid_3,1);
    //右侧部分1：模式选择
    QGroupBox* gbox_mode =new QGroupBox;//内部自动互斥
    gbox_mode->setTitle("模式选择");
    rbtn_simple = new QRadioButton;rbtn_simple->setText("示教模式");rbtn_simple->setChecked(true);//初始模式
    rbtn_target = new QRadioButton;rbtn_target->setText("目标模式");
    rbtn_work = new QRadioButton;rbtn_work->setText("工作模式");rbtn_work->setDisabled(true);//选择算法时的执行状态，不可直接选中
    QVBoxLayout* lay_right_1 = new QVBoxLayout;
    lay_right_1->addWidget(rbtn_simple,1);
    lay_right_1->addWidget(rbtn_target,1);
    lay_right_1->addWidget(rbtn_work,1);
    gbox_mode->setLayout(lay_right_1);
    //右侧部分2：关节控制
    gbox_joint= new QGroupBox;
    gbox_joint->setTitle("关节控制");
    QLabel* lb_ctljoint1 = new QLabel;lb_ctljoint1->setText("关节一");lb_ctljoint1->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    QLabel* lb_ctljoint2 = new QLabel;lb_ctljoint2->setText("关节二");lb_ctljoint2->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    QLabel* lb_ctljoint3 = new QLabel;lb_ctljoint3->setText("关节三");lb_ctljoint3->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    QLabel* lb_ctljoint4 = new QLabel;lb_ctljoint4->setText("关节四");lb_ctljoint4->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    QLabel* lb_ctljoint5 = new QLabel;lb_ctljoint5->setText("关节五");lb_ctljoint5->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    slider_q1 = new QSlider;slider_q1->setOrientation(Qt::Horizontal);
    slider_q2 = new QSlider;slider_q2->setOrientation(Qt::Horizontal);
    slider_q3 = new QSlider;slider_q3->setOrientation(Qt::Horizontal);
    slider_q4 = new QSlider;slider_q4->setOrientation(Qt::Horizontal);
    slider_q5 = new QSlider;slider_q5->setOrientation(Qt::Horizontal);
    slider_q1->setMinimum(0);slider_q1->setMaximum(70);slider_q1->setValue(20);
    slider_q2->setMinimum(0);slider_q2->setMaximum(45);slider_q2->setValue(20);
    slider_q3->setMinimum(0);slider_q3->setMaximum(360);slider_q3->setValue(30);
    slider_q4->setMinimum(0);slider_q4->setMaximum(360);slider_q4->setValue(30);
    slider_q5->setMinimum(0);slider_q5->setMaximum(360);slider_q5->setValue(30);
    QGridLayout* lay_right_2 = new QGridLayout;
    lay_right_2->setColumnStretch(0,4);
    lay_right_2->setColumnStretch(1,10);
    lay_right_2->addWidget(lb_ctljoint1,0,0);
    lay_right_2->addWidget(lb_ctljoint2,1,0);
    lay_right_2->addWidget(lb_ctljoint3,2,0);
    lay_right_2->addWidget(lb_ctljoint4,3,0);
    lay_right_2->addWidget(lb_ctljoint5,4,0);
    lay_right_2->addWidget(slider_q1,0,1);
    lay_right_2->addWidget(slider_q2,1,1);
    lay_right_2->addWidget(slider_q3,2,1);
    lay_right_2->addWidget(slider_q4,3,1);
    lay_right_2->addWidget(slider_q5,4,1);
    gbox_joint->setLayout(lay_right_2);
    //右侧部分3：末端设置
    gbox_target = new QGroupBox;
    gbox_target->setTitle("末端控制");gbox_target->setDisabled(true);
    QLabel* lb_ctlpd = new QLabel;lb_ctlpd->setText("侧位移");lb_ctlpd->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    QLabel* lb_ctlpx = new QLabel;lb_ctlpx->setText("x");lb_ctlpx->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    QLabel* lb_ctlpy = new QLabel;lb_ctlpy->setText("y");lb_ctlpy->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    QLabel* lb_ctlpz = new QLabel;lb_ctlpz->setText("z");lb_ctlpz->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    QLabel* lb_ctlpth= new QLabel;lb_ctlpth->setText("转角");lb_ctlpth->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    edit_pd = new QLineEdit;edit_pd->setText(QString::number(0.28));
    edit_px = new QLineEdit;edit_px->setText(QString::number(-0.25));
    edit_py = new QLineEdit;edit_py->setText(QString::number(0.5));
    edit_pz = new QLineEdit;edit_pz->setText(QString::number(0.1));
    edit_pth = new QLineEdit;edit_pth->setText(QString::number(221));   
    btn_settarget = new QPushButton;btn_settarget->setText("设置\n末端");
    btn_settarget->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    QGridLayout* lay_right_3 = new QGridLayout;
    lay_right_3->setColumnStretch(0,4);
    lay_right_3->setColumnStretch(1,5);
    lay_right_3->setColumnStretch(2,5);
    lay_right_3->addWidget(lb_ctlpd,0,0);
    lay_right_3->addWidget(lb_ctlpx,1,0);
    lay_right_3->addWidget(lb_ctlpy,2,0);
    lay_right_3->addWidget(lb_ctlpz,3,0);
    lay_right_3->addWidget(lb_ctlpth,4,0);
    lay_right_3->addWidget(edit_pd,0,1);
    lay_right_3->addWidget(edit_px,1,1);
    lay_right_3->addWidget(edit_py,2,1);
    lay_right_3->addWidget(edit_pz,3,1);
    lay_right_3->addWidget(edit_pth,4,1);
    lay_right_3->addWidget(btn_settarget,0,2,5,1);
    gbox_target->setLayout(lay_right_3);
    QVBoxLayout* lay_right = new QVBoxLayout;
    lay_right->addWidget(gbox_mode,2);
    lay_right->addWidget(gbox_joint,3);
    lay_right->addWidget(gbox_target,3);
    //总布局
    QHBoxLayout* lay_all = new QHBoxLayout;
    lay_all->addLayout(lay_left,1);
    lay_all->addLayout(lay_mid,3);
    lay_all->addLayout(lay_right,1);
    wgt0->setLayout(lay_all);

    connect(slider_pitch,&QSlider::valueChanged,this,&MainWindow::diy_verticalSliderPitch_valueChanged);
    connect(slider_yaw,&QSlider::valueChanged,this,&MainWindow::diy_horizontalSliderYaw_valueChanged);
    connect(slider_q1,&QSlider::valueChanged,this,&MainWindow::diy_horizontalSliderq1_valueChanged);
    connect(slider_q2,&QSlider::valueChanged,this,&MainWindow::diy_horizontalSliderq2_valueChanged);
    connect(slider_q3,&QSlider::valueChanged,this,&MainWindow::diy_horizontalSliderq3_valueChanged);
    connect(slider_q4,&QSlider::valueChanged,this,&MainWindow::diy_horizontalSliderq4_valueChanged);
    connect(slider_q5,&QSlider::valueChanged,this,&MainWindow::diy_horizontalSliderq5_valueChanged);
    connect(btn_settarget,&QPushButton::clicked,this,&MainWindow::diy_commandLinkButtonGo_clicked);
    connect(rbtn_simple,&QPushButton::clicked,this,&MainWindow::diy_radioButtonSimple_clicked);
    connect(rbtn_target,&QPushButton::clicked,this,&MainWindow::diy_radioButtonTarget_clicked);

    //与文件管理界面交互相关
    subwgt_file = new wgt_manageFile(this);subwgt_file->hide();subwgt_file->setWindowModality(Qt::WindowModality::WindowModal);
    //接受反馈文本并显示
    connect(subwgt_file,&wgt_manageFile::sendInfo_fromwgtfile,this,&MainWindow::showStatusInfo);
    //接收插补后的序列并开始仿真
    connect(subwgt_file,&wgt_manageFile::sendSeqPoseJoint,this,&MainWindow::startSim);
    //发送命令：保存单个点位
    connect(btn_addposeend,&QPushButton::clicked,this,[=]() {
        emit sendPoseEend(robot->getDL(),robot->getX(),robot->getY(),robot->getZ(),robot->getTH());
    });
    connect(this,&MainWindow::sendPoseEend,subwgt_file,&wgt_manageFile::addPoseEndtothis);
    //发送命令：保存路径到本地
    connect(btn_writeseq,&QPushButton::clicked,this,[=]() {
        emit sendInfoSeq(edit_name->text(),edit_attri->text().toInt());
        edit_name->setText("undefied");edit_attri->setText("0");//初始化，防止空值
    });
    connect(this,&MainWindow::sendInfoSeq,subwgt_file,&wgt_manageFile::writeFiletoLocal);
    //打开文件管理窗口
    connect(btn_openlib,&QPushButton::clicked,this,[=]() {
        // 移动子窗口到中央显示
        QPoint globalPos = this->mapToGlobal(QPoint(0,0));//父窗口绝对坐标
        int x = globalPos.x() + (this->width() - subwgt_file->width()) / 2;//x坐标
        int y = globalPos.y() + (this->height() - subwgt_file->height()) / 2;//y坐标
        subwgt_file->move(x, y);
        subwgt_file->show();        
    });
}


void MainWindow::startSim(Seq_PoseJoint *seq_in)
{
    seq_jointforwork = seq_in;
    //进入工作模式
    rbtn_work->setChecked(true);
    robot->setMode(WORKING);
    gbox_joint->setDisabled(true);
    gbox_target->setDisabled(true);
}


