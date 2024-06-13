#include "scara.h"

#include <iostream>

Scara *robot;

Scara::Scara(QObject *parent) :
    QObject(parent),
    q1(0.2), q2(0.200), q3(30),q4(30),q5(30),
    dq1(0.0), dq2(0.0), dq3(0.0),dq4(0.0),dq5(0.0),
    posx(0.0), posy(0.0), posz(0.0),posTH(0.0),
    tDL(0.28),tx(-0.25), ty(0.5), tz(0.100),tTH(221),
    mode(SIMPLE),
    r1(0.085), r2(0.302),r3(0.289),h1(0.061),h2(0.0525),h3(0.038)//初值：米，度
{
    delta_h=h1+h2+h3;
    baseModel.load("base.obj");  
    arm1Model.load("arm1.obj");
    arm2Model.load("arm2.obj");
    arm3Model.load("arm3.obj");
    arm4Model.load("arm4.obj");
    arm5Model.load("arm5.obj");
    kinPr();
}

void Scara::go(double dt)
{
    if(mode == TARGET) {
        //剩下的总位移量
        double rDL = tDL - posDL;
        double rpx = tx - posx;
        double rpy = ty - posy;
        double rpz = tz - posz;
        double rtheta = tTH - posTH;

        // 末端速度
        double vmax_line = 0.1;
        double vmax_theta = 0.5;

        //单次关节变化，即关节速度
        double l_part = sqrt((rpx+rDL)*(rpx+rDL) + rpy*rpy + rpz*rpz); // 局部总位移
        double vd_lateral = rDL*vmax_line/l_part;
        double vpx = rpx * vmax_line/l_part;
        double vpy = rpy * vmax_line/l_part;
        double vpz = rpz * vmax_line/l_part;
        double vtheta = rtheta*vmax_theta/sqrt(rtheta*rtheta);
        calcSpeed(vd_lateral, vpx, vpy, vpz, vtheta); // 计算关节速度dqi并保存到类中
    } else {
        dq1 = 0.0;
        dq2 = 0.0;
        dq3 = 0.0;
        dq4 = 0.0;
        dq5 = 0.0;
    }
    // 运动dt
    q1 += dt * dq1;
    q2 += dt * dq2;
    q3 += dt * dq3;
    q4 += dt * dq4;
    q5 += dt * dq5;

    if(q3 > 360.0) q3 -= 360.0;
    if(q3 < 0.0) q3 += 360.0;
    if(q4 > 360.0) q4 -= 360.0;
    if(q4 < 0.0) q4 += 360.0;
    if(q5 > 360.0) q5 -= 360.0;
    if(q5 < 0.0) q5 += 360.0;

    if(q1 < 0.0) q1 = 0.0;
    if(q1 > 0.7) q1 = 0.7;
    if(q2 < 0.0) q2 = 0.0;
    if(q2 > 0.447) q2 = 0.447;

    kinPr();

    //这三行考虑删掉
    PathPoint p = { posx, posy, posz };
    path.push_back(p);
    if(path.size() > 10000) path.erase(path.begin());

    //综上所述：仅当目标模式下，模型的参数变化由scara类自己决定。在其余模式下，dq=0，即scara类不由自己内部改变参数，而交由母窗口控制。
}

//考虑删掉：
void Scara::displayPath()
{
    std::vector<PathPoint>::iterator i;
    glBegin(GL_LINE_STRIP);
        for(i = path.begin(); i < path.end(); ++i)
            glVertex3f(-(i->y), i->x, i->z+0.5);
    glEnd();
}

void Scara::display()
{
    //绘图——glwidget的paintGL调用
    baseModel.display();
    glTranslatef(0.0, q1, 0.0);//q1
    arm1Model.display();

    glTranslatef(0.0, 0.0, 0.0229);
    glTranslatef(0.0, 0.0, 0.031);//修正
    glTranslatef(0.0, 0.0, q2);//q2
    arm2Model.display();

    glTranslatef(-r1,0.0,0.0);
    glTranslatef(0.0,0.0,-h1);
    glRotatef(-90, 0.0, 0.0, 1.0);//修正
    glRotatef(q3, 0.0, 0.0, 1.0);//q3
    arm3Model.display();

    glTranslatef(-r2,0.0,0.0);
    glTranslatef(0.0,0.0,-h2);
    glRotatef(q4, 0.0, 0.0, 1.0);//q3
    arm4Model.display();

    glTranslatef(-r3,0.0,0.0);
    glTranslatef(0.0,0.0,-h3);
    glRotatef(q5, 0.0, 0.0, 1.0);//q3
    arm5Model.display();
}

double Scara::getq1()
{
    return q1;
}
double Scara::getq2()
{
    return q2;
}
double Scara::getq3()
{
    return q3;
}
double Scara::getq4()
{
    return q4;
}
double Scara::getq5()
{
    return q5;
}

double Scara::getDL()
{
    return posDL;
}
double Scara::getX()
{
    return posx;
}
double Scara::getY()
{
    return posy;
}
double Scara::getZ()
{
    return posz;
}
double Scara::getTH()
{
    return posTH;
}

void Scara::setq1(double temp1)
{
    q1 = temp1;
    kinPr();
}
void Scara::setq2(double temp2)
{
    q2 = temp2;
    kinPr();
}
void Scara::setq3(double temp3)
{
    q3 = temp3;
    kinPr();
}
void Scara::setq4(double temp4)
{
    q4 = temp4;
    kinPr();
}
void Scara::setq5(double temp5)
{
    q5 = temp5;
    kinPr();
}

void Scara::setMode(SteeringMode _mode)
{
    mode = _mode;
}

void Scara::setTarget(double _tDL,double _tx, double _ty, double _tz,double _tTH)
{
    tDL = _tDL;
    tx = _tx;
    ty = _ty;
    tz = _tz;
    tTH = _tTH;
}



void Scara::kinPr()
{
    posDL = q1;
    posx = r2 * cos(M_PI*q3/180.0) + r3 * cos(M_PI*(q3+q4)/180.0);
    posy = r1+r2 * sin(M_PI*q3/180.0) + r3 * sin(M_PI*(q3+q4)/180.0);
    posz = q2-delta_h;
    posTH = q3+q4+q5;
    if(posTH > 360.0) posTH -= 360.0;
    if(posTH < 0.0) posTH += 360.0;
}

void Scara::kinOdwr1(Pose_End* pose_in)
{
    double q1_temp = pose_in->pd;
    double q2_temp = pose_in->pz+delta_h;

    double q0=atan2(pose_in->py-r1,pose_in->px);
    double a0=sqrt((pose_in->py-r1)*(pose_in->py - r1) + (pose_in->px)*(pose_in->px));
    double temp=(a0*a0+(r2)*(r2)-(r3)*(r3))/(2*a0*r2);

    double q3_temp = -acos(temp)+q0;

    temp=(a0*a0-(r2)*(r2)+(r3)*(r3))/(2*a0*r3);
    double q4_temp = acos(temp)+q0-q3_temp;

    q3_temp = q3_temp*180.0/M_PI;
    q4_temp = q4_temp*180.0/M_PI;

    double q5_temp = pose_in->pth - q3_temp - q4_temp;
    if(q3_temp > 360.0) q3_temp -= 360.0;
    if(q3_temp < 0.0) q3_temp += 360.0;
    if(q4_temp > 360.0) q4_temp -= 360.0;
    if(q4_temp < 0.0) q4_temp += 360.0;
    if(q5_temp > 360.0) q5_temp -= 360.0;
    if(q5_temp < 0.0) q5_temp += 360.0;


    setq1(q1_temp);
    setq2(q2_temp);
    setq3(q3_temp);
    setq4(q4_temp);
    setq5(q5_temp);
    kinPr();
}
void Scara::kinOdwr2(Pose_End* pose_in)
{
    double q1_temp = pose_in->pd;
    double q2_temp = pose_in->pz+delta_h;

    double q0=atan2(pose_in->py-r1,pose_in->px);
    double a0=sqrt((pose_in->py-r1)*(pose_in->py - r1) + (pose_in->px)*(pose_in->px));
    double temp=(a0*a0+(r2)*(r2)-(r3)*(r3))/(2*a0*r2);

    double q3_temp = acos(temp)+q0;

    temp=(a0*a0-(r2)*(r2)+(r3)*(r3))/(2*a0*r3);
    double q4_temp = -acos(temp)+q0-q3_temp;

    q3_temp = q3_temp*180.0/M_PI;
    q4_temp = q4_temp*180.0/M_PI;

    double q5_temp = pose_in->pth - q3_temp - q4_temp;
    if(q3_temp > 360.0) q3_temp -= 360.0;
    if(q3_temp < 0.0) q3_temp += 360.0;
    if(q4_temp > 360.0) q4_temp -= 360.0;
    if(q4_temp < 0.0) q4_temp += 360.0;
    if(q5_temp > 360.0) q5_temp -= 360.0;
    if(q5_temp < 0.0) q5_temp += 360.0;

    setq1(q1_temp);
    setq2(q2_temp);
    setq3(q3_temp);
    setq4(q4_temp);
    setq5(q5_temp);
    kinPr();
}
bool Scara::calcSpeed(double vDL,double vx, double vy, double vz,double vTH)
{    
    // 二连杆机器人模型theta1=q3,theta2=q4
    double sth1 = sin(M_PI*q3/180.0);
    double cth1 = cos(M_PI*q3/180.0);
    double sth12 = sin(M_PI*(q3+q4)/180.0);
    double cth12 = cos(M_PI*(q3+q4)/180.0);
    // jacobian矩阵
    double det = (-r2*sth1-r3*sth12) * (r3*cth12) + (r3*sth12) * (r2*cth1+r3*cth12);
    if(!det) {  // 奇异
        dq1 = 0.0;
        dq2 = 0.0;
        dq3 = 0.0;
        dq4 = 0.0;
        dq5 = 0.0;

        return false;
    }
    double t11 = r3*cth12/det;
    double t12 = r3*sth12/det;
    double t21 = -(r2*cth1+r3*cth12)/det;
    double t22 = -(r2*sth1+r3*sth12)/det;

    dq1 = vDL;
    dq2 = vz;
    dq3 = t11*vx + t12*vy;
    dq4 = t21*vx + t22*vy;
    dq5 = vTH-dq3-dq4;

    dq3 *= 180.0/M_PI;
    dq4 *= 180.0/M_PI;
    dq5 *= 180.0/M_PI;

    return true;
}



