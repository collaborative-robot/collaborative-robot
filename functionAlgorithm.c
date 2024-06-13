#include "functionAlgorithm.h"
#include "stdio.h"

//B样条区：
double fun_node(double i,double n)
{
    //准均匀三次
    double step = 1/(n-2);
    if(i<=3){
        return 0;}
    else if(i>=(n+1)){
        return 1;}
    else {
        return (i-3)*step;
    }
}
double divideZero(double up, double down)
{
    if(down<(1e-15)){
        return 0;
    }
    else{
        return up/down;
    }
}
double fun_deboorbase(double i, double k, double x,int n)
{
    if(k==0){
        if((x>=fun_node(i,n))&&(x<=fun_node(i+1,n))){return 1;}
        else{return 0;}
    }
    else{
        return divideZero(x-fun_node(i,n),fun_node(i+k,n)-fun_node(i,n))*fun_deboorbase(i,k-1,x,n)+
                divideZero(fun_node(i+k+1,n)-x,fun_node(i+k+1,n)-fun_node(i+1,n))*fun_deboorbase(i+1,k-1,x,n);
    }
}
double fun_difdeboorbase(double i, double k, double x, int n)
{
    double d1 = divideZero(fun_deboorbase(i,k-1,x,n),(fun_node(i+k,n)-fun_node(i,n)));
    double d2 = divideZero(fun_deboorbase(i+1,k-1,x,n),(fun_node(i+k+1,n)-fun_node(i+1,n)));
    double d = k*(d1-d2);
    return d;
}
Seq_PoseJoint *fun_deboortest(Seq_PoseEnd *seq_in)
{
    //准备工作
    int num_ctlpoint = seq_in->cnt_now-1;
    double startpoint = fun_node(3,num_ctlpoint);
    double endpoint = fun_node(num_ctlpoint+1,num_ctlpoint);
    Seq_PoseJoint* seqposejoint_out = generateSeqPoseJoint();
    double u=startpoint;
    double du = 0;
    double q1=0;
    double q2=0;
    double q3=0;
    double q4=0;
    double q5=0;
    double r1=0.085; double r2=0.302;double r3=0.289;double h1=0.061;double h2=0.0525;double h3=0.038;
    double delta_h=h1+h2+h3;
//    for(int k= 0;startpoint + 0.005*k<endpoint;k++){
//        double u = startpoint + 0.005*k;
//        Pose_End* pose_temp = fun_pose_gen();
//        pose_temp->pd=pose_temp->px=pose_temp->py=pose_temp->pz=pose_temp->pth=0;
//        for(int kk=0;kk<(num_ctlpoint+1);kk++){
//            pose_temp->pd = pose_temp->pd + seq_in->pose_end_point[kk]->pd*fun_deboorbase(kk,3,u,num_ctlpoint);
//            pose_temp->px=pose_temp->px+seq_in->pose_end_point[kk]->pd*fun_deboorbase(kk,3,u,num_ctlpoint);
//            pose_temp->py=pose_temp->py+seq_in->pose_end_point[kk]->pd*fun_deboorbase(kk,3,u,num_ctlpoint);
//            pose_temp->pz=pose_temp->pz+seq_in->pose_end_point[kk]->pd*fun_deboorbase(kk,3,u,num_ctlpoint);
//            pose_temp->pth=pose_temp->pth+seq_in->pose_end_point[kk]->pd*fun_deboorbase(kk,3,u,num_ctlpoint);

//        }
//        fun_pose_add(joint_out,pose_temp);
//    }
//    return joint_out;

    while (u<=endpoint) {
        Pose_Joint* joint_temp = generatePoseJoint();
        double pd_temp=0;
        double px_temp=0;
        double py_temp=0;
        double pz_temp=0;
        double pth_temp=0;
        double dpd_temp=0;
        double dpx_temp=0;
        double dpy_temp=0;
        double dpz_temp=0;
        double dpth_temp=0;

        for(int kk=0;kk<(num_ctlpoint+1);kk++){
            pd_temp = pd_temp + seq_in->pointer_pose_end[kk]->pd*fun_deboorbase(kk,3,u,num_ctlpoint);
            px_temp = px_temp+seq_in->pointer_pose_end[kk]->px*fun_deboorbase(kk,3,u,num_ctlpoint);
            py_temp = py_temp+seq_in->pointer_pose_end[kk]->py*fun_deboorbase(kk,3,u,num_ctlpoint);
            pz_temp = pz_temp+seq_in->pointer_pose_end[kk]->pz*fun_deboorbase(kk,3,u,num_ctlpoint);
            pth_temp = pth_temp+seq_in->pointer_pose_end[kk]->pth*fun_deboorbase(kk,3,u,num_ctlpoint);

            dpd_temp = dpd_temp + seq_in->pointer_pose_end[kk]->pd*fun_difdeboorbase(kk,3,u,num_ctlpoint);
            dpx_temp = dpx_temp+seq_in->pointer_pose_end[kk]->px*fun_difdeboorbase(kk,3,u,num_ctlpoint);
            dpy_temp = dpy_temp+seq_in->pointer_pose_end[kk]->py*fun_difdeboorbase(kk,3,u,num_ctlpoint);
            dpz_temp = dpz_temp+seq_in->pointer_pose_end[kk]->pz*fun_difdeboorbase(kk,3,u,num_ctlpoint);
            dpth_temp = dpth_temp+seq_in->pointer_pose_end[kk]->pth*fun_difdeboorbase(kk,3,u,num_ctlpoint);
        }

        //反解
        if(u==startpoint){
            double q1_temp = pd_temp;
            double q2_temp = pz_temp + delta_h;
            double q0=atan2(py_temp - r1,px_temp);
            double a0=sqrt((py_temp-r1)*(py_temp - r1) + (px_temp)*(px_temp));
            double temp=(a0*a0+(r2)*(r2)-(r3)*(r3))/(2*a0*r2);
            double q3_temp = -acos(temp)+q0;
            temp=(a0*a0-(r2)*(r2)+(r3)*(r3))/(2*a0*r3);
            double q4_temp = acos(temp)+q0-q3_temp;
            q3_temp = q3_temp*180.0/M_PI;
            q4_temp = q4_temp*180.0/M_PI;
            double q5_temp = pth_temp - q3_temp - q4_temp;
            if(q3_temp > 360.0) q3_temp -= 360.0;
            if(q3_temp < 0.0) q3_temp += 360.0;
            if(q4_temp > 360.0) q4_temp -= 360.0;
            if(q4_temp < 0.0) q4_temp += 360.0;
            if(q5_temp > 360.0) q5_temp -= 360.0;
            if(q5_temp < 0.0) q5_temp += 360.0;

            q1=q1_temp;
            q2=q2_temp;
            q3=q3_temp;
            q4=q4_temp;
            q5=q5_temp;
        }
        else {
            //反解1
            double q1_temp1 = pd_temp;
            double q2_temp1 = pz_temp + delta_h;
            double q0=atan2(py_temp - r1,px_temp);
            double a0=sqrt((py_temp-r1)*(py_temp - r1) + (px_temp)*(px_temp));
            double temp=(a0*a0+(r2)*(r2)-(r3)*(r3))/(2*a0*r2);
            double q3_temp1 = -acos(temp)+q0;
            temp=(a0*a0-(r2)*(r2)+(r3)*(r3))/(2*a0*r3);
            double q4_temp1 = acos(temp)+q0-q3_temp1;
            q3_temp1 = q3_temp1*180.0/M_PI;
            q4_temp1 = q4_temp1*180.0/M_PI;
            double q5_temp1 = pth_temp - q3_temp1 - q4_temp1;
            if(q3_temp1 > 360.0) q3_temp1 -= 360.0;
            if(q3_temp1 < 0.0) q3_temp1 += 360.0;
            if(q4_temp1 > 360.0) q4_temp1 -= 360.0;
            if(q4_temp1 < 0.0) q4_temp1 += 360.0;
            if(q5_temp1 > 360.0) q5_temp1 -= 360.0;
            if(q5_temp1 < 0.0) q5_temp1 += 360.0;
            //反解2
            double q1_temp2 = pd_temp;
            double q2_temp2 = pz_temp + delta_h;
            q0=atan2(py_temp - r1,px_temp);
            a0=sqrt((py_temp-r1)*(py_temp - r1) + (px_temp)*(px_temp));
            temp=(a0*a0+(r2)*(r2)-(r3)*(r3))/(2*a0*r2);
            double q3_temp2 = acos(temp)+q0;
            temp=(a0*a0-(r2)*(r2)+(r3)*(r3))/(2*a0*r3);
            double q4_temp2 = -acos(temp)+q0-q3_temp2;
            q3_temp2 = q3_temp2*180.0/M_PI;
            q4_temp2 = q4_temp2*180.0/M_PI;
            double q5_temp2 = pth_temp - q3_temp2 - q4_temp2;
            if(q3_temp2 > 360.0) q3_temp2 -= 360.0;
            if(q3_temp2 < 0.0) q3_temp2 += 360.0;
            if(q4_temp2 > 360.0) q4_temp2 -= 360.0;
            if(q4_temp2 < 0.0) q4_temp2 += 360.0;
            if(q5_temp2 > 360.0) q5_temp2 -= 360.0;
            if(q5_temp2 < 0.0) q5_temp2 += 360.0;
            //比较后得到实际反解
            if(fabs(q3_temp1-q3)<fabs(q3_temp2-q3)){
                q1=q1_temp1;
                q2=q2_temp1;
                q3=q3_temp1;
                q4=q4_temp1;
                q5=q5_temp1;
            }
            else if (fabs(q3_temp1-q3)>fabs(q3_temp2-q3)) {
                q1=q1_temp2;
                q2=q2_temp2;
                q3=q3_temp2;
                q4=q4_temp2;
                q5=q5_temp2;
            }
            else {
                if(fabs(q4_temp1-q4)<fabs(q4_temp2-q4)){
                    q1=q1_temp1;
                    q2=q2_temp1;
                    q3=q3_temp1;
                    q4=q4_temp1;
                    q5=q5_temp1;
                }
                else if (fabs(q4_temp1-q4)>fabs(q4_temp2-q4)) {
                    q1=q1_temp2;
                    q2=q2_temp2;
                    q3=q3_temp2;
                    q4=q4_temp2;
                    q5=q5_temp2;
                }
            }
        }
        //添加关节空间
        joint_temp->q1 = q1;
        joint_temp->q2 = q2;
        joint_temp->q3 = q3;
        joint_temp->q4 = q4;
        joint_temp->q5 = q5;
        addPoseJointtoSeq(seqposejoint_out,joint_temp);
        //计算步长：速度1,时间0.005
        du = 1*0.001*5/sqrt((dpd_temp+dpx_temp)*(dpd_temp+dpx_temp)+dpy_temp*dpy_temp+dpz_temp*dpz_temp);
        u=u+du;
    }
    return seqposejoint_out;
}


