#ifndef FUNCTIONFILE_H
#define FUNCTIONFILE_H

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "malloc.h"
/*
 * ***************************************************************
 * 结构体声明
 * ***************************************************************
*/
//名词命名：例如：序列[主成分]————位置[修饰一（另一种是速度）]————末端[修饰二（另一种是关节）]
//类用大写，实例化对象用小写加角标
typedef struct Pose_End{
    double pd; // 横向滑轨位移量，mm
    double px; // x方向相对横向滑轨偏移，mm
    double py; // y方向相对横向滑轨偏移，mm
    double pz; // z方向相对横向滑轨偏移，mm
    double pth; // 末端旋转角度，°   
}Pose_End;//单个末端空间
//无论这里还是后文，对末端的描述的五个变量一律按照上述顺序，避免混淆
typedef struct Pose_Joint{
    double q1; // 关节一位移量，mm
    double q2; // 关节二位移量，mm
    double q3; // 关节三旋转角度，°
    double q4; // 关节四旋转角度，°
    double q5; // 关节五旋转角度，°
}Pose_Joint;//单个关节空间
typedef struct Seq_PoseEnd{
    Pose_End** pointer_pose_end; //数组指针，该数组存放各末端点位指针
    size_t cnt_max; //最大容量
    size_t cnt_now;//当前数量
    int seq_type;//路径类型参数
    char name[32];//名称
}Seq_PoseEnd;//用于管理某一个json文件中的序列
typedef struct Seq_PoseJoint{
    Pose_Joint** pointer_pose_joint;
    size_t cnt_max; //最大容量
    size_t cnt_now;//当前数量
    int seq_type;
    char name[32];//名称
}Seq_PoseJoint;
//注：目前用到两种序列：末端位置(本地文件保存)以及关节位置（实际运行），每个都是5元数组，这里是分别定义的。
//如果后续还有末端速度和关节速度序列，可否考虑将四种序列统一成一种。
//在序列中的seq_type用不同系数加以区分。(暂时没改)
typedef struct Lib_File{
    Seq_PoseEnd **pointer_seq_pose_end;//数组指针，该数组存放各序列指针
    size_t cnt_max; //最大容量,初始为1
    size_t cnt_now;//当前数量
}Lib_File;//用于管理文件库，即所有json文件。


/*
 * ***************************************************************
 * 函数声明,注：在c++中调用c必须作如下处理（.o中二者对函数的处理不一样）
 * 函数命名：动词（判断用is）+名词+后缀
 * 使用说明：创建序列，创建单个末端；将末端添加到序列；传递序列指针。（是否释放序列看情况）
 * ***************************************************************
*/
#ifdef __cplusplus
extern "C"{
#endif
//生成
Pose_End* generatePoseEnd(void);//生成一个空的末端位置指针
Pose_Joint* generatePoseJoint(void);//生成一个空的关节位置指针
Seq_PoseEnd* generateSeqPoseEnd(void);//生成末端位置序列管理器
Seq_PoseJoint* generateSeqPoseJoint(void);//生成关节位置序列管理器
Lib_File* generateLibFile(void);//生成文件管理器
//添加
void addPoseEndtoSeq(Seq_PoseEnd* temp1,Pose_End* temp2);//向序列管理器动态添加点位
void addPoseJointtoSeq(Seq_PoseJoint* temp1,Pose_Joint* temp2);//向序列管理器动态添加关节
void addSeqPoseEndtoLib(Lib_File* temp1,Seq_PoseEnd* temp2);//向库中添加
//扩容（添加时如果满了会自动按照二倍大小进行扩容，不用手动且扩容次数不会太多）
bool growSeqPoseEnd(Seq_PoseEnd* temp1);//重要！序列动态扩容
bool growSeqPoseJoint(Seq_PoseJoint* temp1);//重要！序列动态扩容
bool growLibFile(Lib_File* temp1);//重要！库动态扩容
//回收
void freeSeqPoseEnd(Seq_PoseEnd* temp1);//回收末端序列管理器
void freeSeqPoseJoint(Seq_PoseJoint* temp1);//回收末端序列管理器
void freeLibFile(Lib_File* temp1);//回收文件管理器
//输出
void showSeqPoseEnd(Seq_PoseEnd* temp1);//打印所有点位

#ifdef __cplusplus
}
#endif


#endif // FUNCTIONFILE_H
