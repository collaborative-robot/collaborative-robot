#include "functionFile.h"

/*
 * ***************************************************************
 * 函数定义
 * ***************************************************************
*/

//生成函数相关
Pose_End* generatePoseEnd(void){
    Pose_End* temp1 = (Pose_End*)calloc(1,sizeof (Pose_End));//or malloc
    if(!temp1){    //记得用free释放
        printf("generate pose fails\n");
        return NULL;
    }
    return temp1;
}
Pose_Joint* generatePoseJoint(void){
    Pose_Joint* temp1 = (Pose_Joint*)calloc(1,sizeof (Pose_Joint));//or malloc
    if(!temp1){    //记得用free释放
        printf("generate joint fails\n");
        return NULL;
    }
    return temp1;
}
Seq_PoseEnd* generateSeqPoseEnd(void){
    Seq_PoseEnd* temp1 = (Seq_PoseEnd*)calloc(1,sizeof (Seq_PoseEnd));
    if(!temp1){    //后面用free释放
        printf("generate sequence fails\n");
        return NULL;
    }
    temp1->cnt_max = 1;//初始化，开辟一个位置，后面动态扩容
    temp1->pointer_pose_end = (Pose_End**)calloc(temp1->cnt_max,sizeof(Pose_End*));
    if(!temp1->pointer_pose_end){
        printf("init sequence fails\n");
        free(temp1);//由于没有生成成功仅free即可
        return NULL;
    }
    return temp1;
}

Seq_PoseJoint* generateSeqPoseJoint(void){
    Seq_PoseJoint* temp1 = (Seq_PoseJoint*)calloc(1,sizeof (Seq_PoseJoint));
    if(!temp1){    //后面用free释放
        printf("generate sequence fails\n");
        return NULL;
    }
    temp1->cnt_max = 1;//初始化，开辟一个位置，后面动态扩容
    temp1->pointer_pose_joint = (Pose_Joint**)calloc(temp1->cnt_max,sizeof(Pose_Joint*));
    if(!temp1->pointer_pose_joint){
        printf("init sequence fails\n");
        free(temp1);//由于没有生成成功仅free即可
        return NULL;
    }
    return temp1;
}
Lib_File* generateLibFile(void){
    Lib_File* temp1 = (Lib_File*)calloc(1,sizeof (Lib_File));
    if(!temp1){
        printf("generate lib fails\n");
        return NULL;
    }
    temp1->cnt_max = 1;//初始化，开辟一个位置，后面动态扩容
    temp1->pointer_seq_pose_end = (Seq_PoseEnd**)calloc(temp1->cnt_max,sizeof(Seq_PoseEnd*));
    if(!temp1->pointer_seq_pose_end){
        printf("init lib fails\n");
        free(temp1);
        return NULL;
    }
    return temp1;
}
//扩容函数
bool growSeqPoseEnd(Seq_PoseEnd* temp1){
    Pose_End** temp2 = (Pose_End**)realloc(temp1->pointer_pose_end,temp1->cnt_max*2*sizeof (Pose_End*));
    temp1->pointer_pose_end = temp2;
    temp1->cnt_max *=2;
    return true;
}
bool growSeqPoseJoint(Seq_PoseJoint* temp1){
    Pose_Joint** temp2 = (Pose_Joint**)realloc(temp1->pointer_pose_joint,temp1->cnt_max*2*sizeof (Pose_Joint*));
    temp1->pointer_pose_joint = temp2;
    temp1->cnt_max *=2;
    return true;
}
bool growLibFile(Lib_File* temp1){
    Seq_PoseEnd** temp2 = (Seq_PoseEnd**)realloc(temp1->pointer_seq_pose_end,temp1->cnt_max*2*sizeof (Seq_PoseEnd*));
    temp1->pointer_seq_pose_end = temp2;
    temp1->cnt_max *=2;
    return true;
}
//回收
void freeSeqPoseEnd(Seq_PoseEnd* temp1){
    for(size_t i =0 ;i<temp1->cnt_now ;i++){
        free(temp1->pointer_pose_end[i]);
    }
    free(temp1->pointer_pose_end);
    free(temp1);
}
void freeSeqPoseJoint(Seq_PoseJoint* temp1){
    for(size_t i =0 ;i<temp1->cnt_now ;i++){
        free(temp1->pointer_pose_joint[i]);
    }
    free(temp1->pointer_pose_joint);
    free(temp1);
}
void freeLibFile(Lib_File* temp1){
    for(size_t i =0 ;i<temp1->cnt_now ;i++){
        freeSeqPoseEnd(temp1->pointer_seq_pose_end[i]);
    }
    free(temp1->pointer_seq_pose_end);
    free(temp1);
}
//添加
void addPoseEndtoSeq(Seq_PoseEnd* temp1,Pose_End* temp2){
    temp1->pointer_pose_end[temp1->cnt_now++] = temp2;
    if(temp1->cnt_now == temp1->cnt_max){
        growSeqPoseEnd(temp1);
    }
}
void addPoseJointtoSeq(Seq_PoseJoint* temp1,Pose_Joint* temp2){
    temp1->pointer_pose_joint[temp1->cnt_now++] = temp2;
    if(temp1->cnt_now == temp1->cnt_max){
        growSeqPoseJoint(temp1);
    }
}
void addSeqPoseEndtoLib(Lib_File* temp1,Seq_PoseEnd* temp2){
    temp1->pointer_seq_pose_end[temp1->cnt_now++] = temp2;
    if(temp1->cnt_now == temp1->cnt_max){
        growLibFile(temp1);
    }
}
//展示序列所有点位,测试用
void showSeqPoseEnd(Seq_PoseEnd* temp1){
    for (size_t i= 0;i < temp1->cnt_now;i++){
       printf("%zu: %f %f %f %f %f\n",i,temp1->pointer_pose_end[i]->px,temp1->pointer_pose_end[i]->py,
              temp1->pointer_pose_end[i]->pz,temp1->pointer_pose_end[i]->pth,temp1->pointer_pose_end[i]->pd);
    }
    int j=temp1->cnt_now;
    printf("%d",j);
}
