#include "comm.h"
#include "control.h"
#include "map_nav.h"
#include <stdio.h>
#include <chrono>
#include <thread>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#define IP "127.0.0.1"//"192.168.1.228"
#define PORT 8888

out_data od;
in_state is;
move_para mp;

void decide_all(pi_comm *comm, controller *cont);

int main(int argc, char *argv[]){

    ros::init(argc, argv, "henry");
    
    ros::NodeHandle n1, n2;
    
    pi_comm comm = pi_comm(IP, PORT);

    controller cont = controller(&n1);
    
    map_nav mn = map_nav(&n1, &n2);

    ros::Rate r(10);
    
    while(1){
        decide_all(&comm, &cont);
        r.sleep();
    }

    return 0;
}

void decide_all(pi_comm *comm, controller *cont){
    is = comm->read_data();

    if(is.W == 1 && is.S == 0){
        od.os.W = 1;
        mp.W = 1;
        od.os.S = 0;
        mp.S = 0;
    } else if(is.S == 1 && is.W == 0){
        od.os.S = 1;
        mp.S = 1;
        od.os.W = 0;
        mp.W = 0;
    } else {
        od.os.W = 0;
        mp.W = 0;
        od.os.S = 0;
        mp.S = 0;
    }

    if(is.A == 1 && is.D == 0){
        od.os.A = 1;
        mp.A = 1;
        od.os.D = 0;
        mp.D = 0;
    } else if(is.D == 1 && is.A == 0){
        od.os.D = 1;
        mp.D = 1;
        od.os.A = 0;
        mp.A = 0;
    } else {
        od.os.A = 0;
        mp.A = 0;
        od.os.D = 0;
        mp.D = 0;
    }

    cont->write_mp(mp);

    comm->write_od(&od);
}
