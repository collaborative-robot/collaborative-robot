#ifndef PAR_KINEMATICS_H
#define PAR_KINEMATICS_H
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
/*
 * ***************************************************************
 * 通用结构体区域,视情况添加
 * ***************************************************************
*/
typedef struct Pose_End{
    double px; // x方向相对横向滑轨偏移，mm
    double py; // y方向相对横向滑轨偏移，mm
    double pz; // z方向相对横向滑轨偏移，mm
    double theta1; // 末端旋转角度，°
    double q5; // 横向滑轨位移量
}Pose_End;//单个末端位姿

typedef struct Pose_End_Seq{
    Pose_End** pose_end_point; //数组指针，该数组存放各末端点位指针
    size_t cnt_max; //最大容量
    size_t cnt_now;//当前数量
    int seq_type;//路径类型，目前只有一种：一般末端序列。后续类型变多后添加文档说明。
}Pose_End_Seq;//用于管理某一个json文件中的序列

typedef struct Seq_lib{
    Pose_End_Seq** pose_end_seq_point;//数组指针，该数组存放各序列指针
    size_t cnt_max; //最大容量
    size_t cnt_now;//当前数量
}Seq_lib;//用于管理序列库，即所有json文件。
/*
 * **************************************************************
 * 通用函数区(整个项目都可能会用)
 * **************************************************************
*/
Pose_End* fun_pose_gen(void);//生成一个空的末端位姿指针
Pose_End_Seq* fun_seq_gen(void);//生成末端序列管理器
void fun_seq_free(Pose_End_Seq*);//回收末端序列管理器
void fun_pose_add(Pose_End_Seq* temp1,Pose_End* temp2);//向序列管理器动态添加点位
bool fun_seq_grow(Pose_End_Seq* temp1);//重要！重要！重要！序列动态扩容

Seq_lib* fun_lib_gen(void);//生成文件管理器
void fun_lib_free(Seq_lib* temp1);//回收文件管理器


/*
 * **************************************************************
 * 文件管理系统函数区（仅在json文件功能测试时使用）
 * **************************************************************
 */
int xin_menu();//菜单界面
Pose_End* fun_pose_diy(void);//手动输入末端位姿
void fun_seq_show(Pose_End_Seq* temp1);//打印所有点位

#endif // PAR_KINEMATICS_H
