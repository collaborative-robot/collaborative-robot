#include <stdio.h>
#include <math.h>

typedef struct {
   double q1; //水平方向滑轨J1移动量，单位：mm （与基座相连）
   double q2; //竖直方向滑轨J2移动量，单位：mm （与J1相连）
   double q3; //关节J3旋转量，单位：°（与J2相连）
   double q4; //关节J4旋转量，单位：°（与J3相连）
   double q5; //关节J5旋转量，单位：°（与J4相连，安装加持工具）
} Robot_Model; //关节位移量

typedef struct 
{
   double l1; // J3长度，mm
   double l2; // J4长度, mm
   double l3; // J5长度, mm
} Robot_Rotation_Length; // 机器人旋转关节J3,J4,J5尺寸

typedef struct {
   double d_lateral; // 横向滑轨位移量，mm
   double px; // x方向相对横向滑轨偏移，mm
   double py; // y方向相对横向滑轨偏移，mm
   double pz; // z方向相对横向滑轨偏移，mm
   double theta; // 末端旋转角度，°
} Pose_End; // 机器人末端位姿

// 逆运动学，根据末端位姿求关节位移量
void ikine(Pose_End* pose_end, Robot_Model *robot_model, Robot_Rotation_Length* length){
   robot_model->q1=pose_end->d_lateral;
   robot_model->q2=pose_end->pz;
   // 此处末端可视为二连杆机器人，逆运动学求解只考虑第二关节角度大于0的情况
   double q0=atan2(pose_end->py-length->l1,pose_end->px);
   double a0=sqrt((pose_end->py-length->l1)*(pose_end->py - length->l1) + (pose_end->px)*(pose_end->px));
   double temp=(a0*a0+(length->l2)*(length->l2)-(length->l3)*(length->l3))/(2*a0*length->l2);
   robot_model->q3=-acos(temp)+q0;
   temp=(a0*a0-(length->l2)*(length->l2)+(length->l3)*(length->l3))/(2*a0*length->l3);
   robot_model->q4=acos(temp)+q0-robot_model->q3;
   robot_model->q5 = pose_end->theta - robot_model->q3 - robot_model->q4;
}