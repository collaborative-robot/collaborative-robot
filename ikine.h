#ifndef IKINE
#define IKINE
#include <stdio.h>
#include <math.h>
#include "robot_model.h"
#define PI 3.14159265358979323846

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
   robot_model->q3=robot_model->q3*180.0/PI;
   robot_model->q4=robot_model->q4*180.0/PI;
   robot_model->q5 = Angle_Adjustment(pose_end->theta - robot_model->q3 - robot_model->q4);
}

// 正运动学，用于辅助校对
void fkine(Pose_End* pose_end, Robot_Model *robot_model, Robot_Rotation_Length* length){
   pose_end->d_lateral=robot_model->q1;
   pose_end->pz=robot_model->q2;
   pose_end->px=length->l2*cos(robot_model->q3*PI/180.0)+length->l3*cos((robot_model->q3+robot_model->q4)*PI/180.0);
   pose_end->py=length->l1+length->l2*sin(robot_model->q3*PI/180.0)+length->l3*sin((robot_model->q3+robot_model->q4)*PI/180.0);
   pose_end->theta=Angle_Adjustment(robot_model->q3+robot_model->q4+robot_model->q5);
}

#endif