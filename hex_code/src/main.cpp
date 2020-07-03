#include "control.h"
#include "map_nav.h"
#include <stdio.h>
#include <chrono>
#include <thread>
#include "ros/ros.h"

int main(int argc, char *argv[]){

    ros::init(argc, argv, "henry");
    
    ros::NodeHandle n1, n2;

    controller cont = controller(&n1);
    
    map_nav mn = map_nav(&n1, &n2, &cont);

    return 0;
}