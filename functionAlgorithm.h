#ifndef FUNCTIONALGORITHM_H
#define FUNCTIONALGORITHM_H

#include "functionFile.h"
#include "math.h"


#ifdef __cplusplus
extern "C"{
#endif
//B样条用：//算法这块的函数及命名还未优化，待修改
double fun_node(double i,double n);
double divideZero(double up,double down);
double fun_deboorbase(double i,double k,double x,int n);
double fun_difdeboorbase(double i,double k,double x,int n);
Seq_PoseJoint* fun_deboortest(Seq_PoseEnd* seqposeend_in);
//FIR用：
//
//
//
#ifdef __cplusplus
}
#endif




#endif // FUNCTIONALGORITHM_H
