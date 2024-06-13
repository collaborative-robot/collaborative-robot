#ifndef SCARA_H
#define SCARA_H
#include <QObject>
#include "qdebug.h"
#include <cmath>
#include <vector>
#include "qtimer.h"
#include "model.h"
#include "functionFile.h"
/*
 * 机器人模型类，包含四个模型及相应的控制函数，
 * 这里控制函数仅是控制仿真，后续添加硬件的控制驱动函数
 * 注：在本程序中，末端的参数顺序一律按照dl,x,y,z,theta的顺序。
 * 校对完毕；鑫2024.6.13
*/
enum SteeringMode { SIMPLE, TARGET, WORKING };//仿真的三个模式：示教模式（控制关节），目标点位模式（控制末端）和算法模式

class Scara : public QObject
{
    Q_OBJECT
public:
    explicit Scara(QObject *parent = 0);

signals:

public slots:
    void go(double dt);//运行函数，与主界面qtime绑定，计算参数变化过程
    void displayPath();//画出末端位移曲线，正考虑删掉。。
    void display();//根据实时参数建模更新（最重要的函数。。

    double getq1();
    double getq2();
    double getq3();
    double getq4();
    double getq5();//获取关节角度
    double getDL();
    double getX();
    double getY();
    double getZ();//获取末端位置
    double getTH();
    void setq1(double temp1);
    void setq2(double temp2);
    void setq3(double temp3);
    void setq4(double temp4);
    void setq5(double temp5);//设置关节角度并计算末端
    void setMode(SteeringMode mode);
    void setTarget(double tDL,double tx, double ty, double tz,double tTH);//设置目标末端
    void kinOdwr1(Pose_End* pose_in);//逆运动学1,这里虽然写了，不过没用上
    void kinOdwr2(Pose_End* pose_in);//2

private slots:
    void kinPr();//正运动学计算末端位姿
    bool calcSpeed(double vDL,double vx, double vy, double vz,double vTH);


private:
    double q1, q2, q3,q4,q5;//关节变化量
    double dq1, dq2, dq3,dq4,dq5;//关节在单位时间变化，实质是速度
    double posDL,posx, posy, posz,posTH;//当前末端位姿
    double tDL,tx, ty, tz,tTH;//目标末端

    SteeringMode mode;

    double r1, r2,r3,delta_h,h1,h2,h3;//3个臂长，第四个是夹持用，这里随意定的，不影响结果

    Model baseModel, arm1Model,arm2Model, arm3Model, arm4Model,arm5Model;//模型块

    struct PathPoint { double x, y, z; };//
    std::vector<PathPoint> path;//末端位置序列//遗留函数，在本项目中没有必要

};

extern Scara *robot;//该类对应的一个全局变量

#endif // SCARA_H
