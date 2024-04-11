#ifndef ROBOT_MODEL
#define ROBOT_MODEL
#include <stdio.h>
#include <math.h>

typedef struct {
   double l1; // J3长度，mm
   double l2; // J4长度, mm
   double l3; // J5长度, mm
} Robot_Rotation_Length; // 机器人旋转关节J3,J4,J5尺寸

typedef struct {
   double q1; //水平方向滑轨J1移动量，单位：mm （与基座相连）
   double q2; //竖直方向滑轨J2移动量，单位：mm （与J1相连）
   double q3; //关节J3旋转量，单位：°（与J2相连）
   double q4; //关节J4旋转量，单位：°（与J3相连）
   double q5; //关节J5旋转量，单位：°（与J4相连，安装加持工具）
} Robot_Model; //关节位移量

typedef struct {
   double d_lateral; // 横向滑轨位移量，mm
   double px; // x方向相对横向滑轨偏移，mm
   double py; // y方向相对横向滑轨偏移，mm
   double pz; // z方向相对横向滑轨偏移，mm
   double theta; // 末端旋转角度，°
} Pose_End; // 机器人末端位姿

// 设置机器人旋转关节J3,J4,J5连杆长度
void Set_Rotation_Length(Robot_Rotation_Length* robot_rotation_length,double l1,double l2,double l3){
   robot_rotation_length->l1=l1;
   robot_rotation_length->l2=l2;
   robot_rotation_length->l3=l3;
}

// 设置机器人关节位移量q1--q5
void Set_Robot_Model(Robot_Model *robot_model,double q1,double q2,double q3,double q4,double q5){
   robot_model->q1=q1;
   robot_model->q2=q2;
   robot_model->q3=q3;
   robot_model->q4=q4;
   robot_model->q5=q5;
}

// 查看机器人旋转关节J3,J4,J5连杆长度
void Show_Rotation_Length(Robot_Rotation_Length* robot_rotation_length){
   printf("机器人旋转关节杆长为：\n");
   printf("l1 = %f, l2 = %f , l3 = %f \n",robot_rotation_length->l1,robot_rotation_length->l2,robot_rotation_length->l3);
}

// 查看机器人关节位移量q1--q5
void Show_Robot_Model(Robot_Model *robot_model){
   printf("机器人关节位移量为：\n");
   printf("q1 = %f\n",robot_model->q1);
   printf("q2 = %f\n",robot_model->q2);
   printf("q3 = %f\n",robot_model->q3);
   printf("q4 = %f\n",robot_model->q4);
   printf("q5 = %f\n",robot_model->q5);
}

// 查看机器人末端位姿
void Show_Pose_End(Pose_End *pose_end){
   printf("机器人末端位姿为：\n");
   printf("d_lateral = %f\n",pose_end->d_lateral);
   printf("px = %f\n",pose_end->px);
   printf("py = %f\n",pose_end->py);
   printf("py = %f\n",pose_end->pz);
   printf("theta = %f\n",pose_end->theta);
}

// 调整角度，角度取值区间(-180°, 180°]
double Angle_Adjustment(double angle){
   if (angle > 180.0) angle-=360.0;
   else if (angle < -180.0 || angle == 180.0) angle+=360.0;   
   return angle;
}

#endif