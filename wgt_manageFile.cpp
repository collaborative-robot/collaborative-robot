#include "wgt_manageFile.h"

wgt_manageFile::wgt_manageFile(QWidget *parent) : QMainWindow(parent)
{
    initWgt();

    connect(btn_hide, &QPushButton::clicked, this, [=]() {
        this->hide();
    });
    connect(btn_run,&QPushButton::clicked,this,&wgt_manageFile::workSLOT);
    seq_test = generateSeqPoseEnd();
    lib_test = generateLibFile();
    btngroup_lib = new QButtonGroup(this);
    readFilefromLocal();
}

wgt_manageFile::~wgt_manageFile()
{
    //析构，释放空间
    freeSeqPoseEnd(seq_test);
    freeLibFile(lib_test);
}

void wgt_manageFile::addPoseEndtothis(double _pd, double _px, double _py, double _pz, double _pth)
{
    //将点位保存到序列中
    Pose_End* pose_test = generatePoseEnd();
    pose_test->pd=_pd;
    pose_test->px=_px;
    pose_test->py=_py;
    pose_test->pz=_pz;
    pose_test->pth=_pth;
    addPoseEndtoSeq(seq_test,pose_test);
    emit sendInfo_fromwgtfile("add pose success");
}

void wgt_manageFile::writeFiletoLocal(QString _name, int _att)
{
    //将seq序列转移到json>>保存到本地>>回收seq内存
    //转移到json
    QJsonObject node1;
    node1.insert("name",_name);
    node1.insert("attribute1",_att);//目前一个，供测试用
    QJsonArray node2;
    for(size_t i=0;i<seq_test->cnt_now;i++){
        QJsonArray node3;
        node3.append(seq_test->pointer_pose_end[i]->pd);
        node3.append(seq_test->pointer_pose_end[i]->px);
        node3.append(seq_test->pointer_pose_end[i]->py);
        node3.append(seq_test->pointer_pose_end[i]->pz);
        node3.append(seq_test->pointer_pose_end[i]->pth);
        node2.append(node3);
    }
    node1.insert("data",node2);
    //保存到本地library_pose文件夹下
    QJsonDocument doc(node1);
    QByteArray js = doc.toJson();
    QDir dir;
    if (!dir.exists("./library_pose")) {
        dir.mkdir("./library_pose");
    }
    QString filepath = "./library_pose/"+_name+".json";
    QFile file(filepath);
    file.open(QFile::WriteOnly);
    file.write(js);
    file.close();
    freeSeqPoseEnd(seq_test);//供下一次使用
    seq_test = generateSeqPoseEnd();
    readFilefromLocal();//重新载入

    emit sendInfo_fromwgtfile("save seq success");
}

void wgt_manageFile::readFilefromLocal()
{
    //先清空，保证无残留
    freeLibFile(lib_test);
    lib_test = generateLibFile();
    clearBtnGroup();
    lay_lib->setSpacing(10);
    //读取文件夹中所有json文件到lib_test中
    QDir dir;
    if (!dir.exists("./library_pose")) {
        qDebug()<<"未找到指定文件夹";
    }
    else{
        QString dirPath = "./library_pose";
        QStringList list;
        QDirIterator it(dirPath, QStringList() << "*.json", QDir::Files | QDir::NoSymLinks | QDir::NoDotAndDotDot, QDirIterator::Subdirectories);
        while(it.hasNext())
        {
            it.next();
            list.append(it.fileInfo().absoluteFilePath());
        }
        if(list.size()==0){qDebug()<<"当前存储路径数：0";}
        else{
            //对每一个路径文件
            foreach(QString str, list)
            {
                QFile file(str);
                file.open(QFile::ReadOnly);
                QByteArray all = file.readAll();
                file.close();

                Seq_PoseEnd* seq_temp = generateSeqPoseEnd();//新的序列指针，必须是新的

                QJsonDocument doc = QJsonDocument::fromJson(all);
                if(doc.isObject()){
                    QJsonObject node1 = doc.object();
                    QStringList keys = node1.keys();
                    for(int i =0;i<keys.size();++i){
                        QString key = keys.at(i);
                        QJsonValue val = node1.value(key);
                        if(val.isString()){
                            const char* ch_temp = val.toString().toStdString().c_str();
                            memcpy(seq_temp->name,ch_temp,32);
                            //qDebug()<<"name add:"<<seq_temp->name;
                            QPushButton* btn_seq = new QPushButton(val.toString());btn_seq->setFixedSize(150,40);btn_seq->setCheckable(true);
                            btngroup_lib->addButton(btn_seq);
                            lay_lib->addWidget(btn_seq,1);
                        }
                        else if (val.isDouble()) {
                            seq_temp->seq_type=val.toInt();
                        }
                        else if (val.isArray()) {
                            QJsonArray node2 = val.toArray();
                            for (int k=0;k<node2.size() ;k++ ) {
                                if(node2.at(k).isArray()){
                                    QJsonArray node3 = node2.at(k).toArray();
                                    Pose_End* pose_temp = generatePoseEnd();
                                    pose_temp->pd=node3.at(0).toDouble();
                                    pose_temp->px=node3.at(1).toDouble();
                                    pose_temp->py=node3.at(2).toDouble();
                                    pose_temp->pz=node3.at(3).toDouble();
                                    pose_temp->pth=node3.at(4).toDouble();
                                    addPoseEndtoSeq(seq_temp,pose_temp);
                                }
                            }
                        }
                    }
                    addSeqPoseEndtoLib(lib_test,seq_temp);
                }
                else {
                    qDebug()<<"格式错误";
                }
            }
            subwgt_scrollarea->setLayout(lay_lib);
            scrollarea_lib->setWidget(subwgt_scrollarea);
        }
    }


    //展示库，仅显示路径点位的第一个点，5个太麻烦了
//    for (size_t i= 0;i < lib_test->cnt_now;i++){
//        qDebug()<<"路径序号"<<i+1;
//        qDebug()<<"名称"<< lib_test->pose_end_seq_point[i]->name<<"属性"<<lib_test->pose_end_seq_point[i]->seq_type;
//        for (size_t k= 0;k < lib_test->pose_end_seq_point[i]->cnt_now;k++){
//            qDebug()<<"pd:"<<k<<lib_test->pose_end_seq_point[i]->pose_end_point[k]->pd;
//        }
//    }

}

void wgt_manageFile::workSLOT()
{
    //找出名字
    QPushButton *selectedButton = qobject_cast<QPushButton*>(btngroup_lib->checkedButton());
    QString str_selected;
    if(selectedButton)
    {
        str_selected = selectedButton->text();
    }
    //找出对应的序列并发送。
    for(size_t k =0;k<lib_test->cnt_now;k++){
        QString str_find = QString(lib_test->pointer_seq_pose_end[k]->name);
        if(str_find==str_selected){
            //选中路径为lib_test->pointer_seq_pose_end[k]  一个Seq_PoseEnd
            //输出seq_final为计算插补后的关节序列，按照5ms间隔依次执行即可（后续也可以有速度序列这些）
            //得到关节变化并发送到主界面。这里的算法过程后面移植到下位机进行
            Seq_PoseJoint* seq_final = fun_deboortest(lib_test->pointer_seq_pose_end[k]);
            emit sendSeqPoseJoint(seq_final);
            //for (size_t kk= 0;kk < seq_final->cnt_now;kk++){
            //    qDebug()<<"pd:"<<k<<seq_final->pose_end_point[kk]->pd;
            //}
        }
    }
    this->hide();
}

void wgt_manageFile::clearBtnGroup()
{
    while (lay_lib->count() > 0) {
        QWidget *widget = lay_lib->itemAt(0)->widget();
        if (widget) {
            lay_lib->removeWidget(widget);
            widget->deleteLater(); // 确保删除控件,程序结束后统一回收
        }
        else{
            lay_lib->takeAt(0);
        }
    }

    QList<QAbstractButton *> buttonList = btngroup_lib->buttons();
    for (int i = 0; i < buttonList.size(); ++i)
    {
        btngroup_lib->removeButton(buttonList[i]);
    }
}

void wgt_manageFile::initWgt(){
    this->setWindowTitle("路径文件");
    QWidget* wgt0 = new QWidget;//qmainwindow没有widget，无法设置布局，需要这样设置。
    this->setCentralWidget(wgt0);
    this->resize(300,400);
    scrollarea_lib = new QScrollArea;scrollarea_lib->setWidgetResizable(true);
    subwgt_scrollarea = new QWidget;
    lay_lib = new QVBoxLayout;lay_lib->setSpacing(2);//创建但暂时不用
    btn_run = new QPushButton;btn_run->setText("运行");btn_run->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    btn_hide = new QPushButton;btn_hide->setText("关闭");btn_hide->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    QGridLayout* lay_all = new QGridLayout;
    lay_all->setColumnStretch(0,2);
    lay_all->setColumnStretch(1,1);
    lay_all->addWidget(scrollarea_lib,0,0,4,1);
    lay_all->addWidget(btn_run,0,1);
    lay_all->addWidget(btn_hide,2,1);
    wgt0->setLayout(lay_all);
}





