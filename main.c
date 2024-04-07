#include <par_kinematics.h>
#include <malloc.h>  //动态成员变量

int main()
{
    //Seq_lib* lib_test = fun_lib_gen();
    //xin_menu();
    printf("%d\n",1);
    //scanf("%d",&t);



    //fun_lib_free(lib_test);
    printf("退出系统成功\n");
    return 0;
}
//初始菜单栏(测试用)OK
int xin_menu(){
    printf("*****末端路径管理系统******\n");
    printf("*    1,导入末端路径文件   *\n");
    printf("*    2,手动添加末端路径文件*\n");
    printf("*    3,查看已有末端路径文件*\n");
    printf("*    4,删除末端路径文件   *\n");
    printf("*    0,退出系统          *\n");
    printf("*************************\n");
    int temp_opt = -1;
    printf("请选择功能：\n");
    scanf("%d",&temp_opt);
    //getchar();
    return temp_opt;
}
//生成末端位姿空指针OK
Pose_End* fun_pose_gen(void){
    Pose_End* temp1 = calloc(1,sizeof (Pose_End));//或者malloc
    if(!temp1){    //记得用free释放
        printf("generating empty pose_end fails\n");
        return NULL;
    }
    return temp1;
}
//键入末端位姿：5个double
Pose_End* fun_pose_diy(void){
    Pose_End* temp1 = fun_pose_gen();
    scanf("%lf %lf %lf %lf %lf",&temp1->px,&temp1->py,&temp1->pz,&temp1->theta1,&temp1->q5);
    getchar();
    return temp1;
}
//生成序列指针（仅管理一个序列，即一个json文件）ok
Pose_End_Seq* fun_seq_gen(void){
    Pose_End_Seq* temp1 = calloc(1,sizeof (Pose_End_Seq));
    if(!temp1){    //后面用free释放
        printf("generate sequence fails\n");
        return NULL;
    }
    temp1->cnt_max = 1;//初始化，开辟一个位置，后面动态扩容
    temp1->pose_end_point = calloc(temp1->cnt_max,sizeof(Pose_End*));
    if(!temp1->pose_end_point){
        printf("init sequence fails\n");
        free(temp1);
        return NULL;
    }
    return temp1;
}
//生成一个文件库ok
Seq_lib* fun_lib_gen(void){
    Seq_lib* temp1 = calloc(1,sizeof (Seq_lib));
    if(!temp1){
        printf("generate lib fails\n");
        return NULL;
    }
    temp1->cnt_max = 1;//初始化，开辟一个位置，后面动态扩容
    temp1->pose_end_seq_point = calloc(temp1->cnt_max,sizeof(Pose_End_Seq*));
    if(!temp1->pose_end_seq_point){
        printf("init lib fails\n");
        free(temp1);
        return NULL;
    }
    return temp1;
}
//回收文件库空间ok
void fun_lib_free(Seq_lib* temp1){
    for(size_t i =0 ;i<temp1->cnt_now ;i++){
        fun_seq_free(temp1->pose_end_seq_point[i]);
    }
    free(temp1->pose_end_seq_point);
    free(temp1);
}
//回收序列空间ok
void fun_seq_free(Pose_End_Seq* temp1){
    for(size_t i =0 ;i<temp1->cnt_now ;i++){
        free(temp1->pose_end_point[i]);
    }
    free(temp1->pose_end_point);
    free(temp1);
}
//向序列添加一个末端ok
void fun_pose_add(Pose_End_Seq* temp1,Pose_End* temp2){
    temp1->pose_end_point[temp1->cnt_now++] = temp2;
}
//动态添加时需要看情况扩容
bool fun_line_grow(Pose_End_Seq* temp1){
    Pose_End** temp2 = realloc(temp1->pose_end_point,temp1->cnt_max*2*sizeof (Pose_End*));
    temp1->pose_end_point = temp2;
    temp1->cnt_max *=2;
    return true;
}
//展示序列所有点位ok
void fun_line_show(Pose_End_Seq* temp1){
    printf("all poses:\n");
    for (size_t i= 0;i < temp1->cnt_now;i++){
        printf("%zu: %f %f %f %f %f\n",i,temp1->pose_end_point[i]->px,temp1->pose_end_point[i]->py,
               temp1->pose_end_point[i]->pz,temp1->pose_end_point[i]->theta1,temp1->pose_end_point[i]->q5);
    }
}

