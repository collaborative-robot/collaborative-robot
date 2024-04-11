#include <stdio.h>
#include "robot_model.h"
#include "ikine.h"

int main(){
    Robot_Rotation_Length robot_rotation_length;
    Set_Rotation_Length(&robot_rotation_length, 10.0, 20.0 , 25.0);
    Show_Rotation_Length(&robot_rotation_length);
    Robot_Model robot_model;
    Set_Robot_Model(&robot_model,10.0, 40.0, 75.0, 60.0, 30.0);
    Show_Robot_Model(&robot_model);
    Pose_End pose_end;
    fkine(&pose_end, &robot_model, &robot_rotation_length);
    Show_Pose_End(&pose_end);
    ikine(&pose_end, &robot_model, &robot_rotation_length);
    Show_Robot_Model(&robot_model);
    return 0;
}