#include "comm.h"
#include "motion.h"
#include "control.h"
#include "laser.h"
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
pos_lidar pl;
obstacle obs;

void decide_all(pi_comm *comm, imu *im, laser *l, controller *cont);

int main(int argc, char *argv[]){

    ros::init(argc, argv, "henry");

    ros::NodeHandle n;
    
    pi_comm comm = pi_comm(IP, PORT);

    imu im;

    controller cont = controller(&n);

    laser l;

    int interval = 20;
    auto time_interval = std::chrono::milliseconds(interval);
    auto time = std::chrono::steady_clock::now();
    
    while(1){
        time += time_interval;
        decide_all(&comm, &im, &l, &cont);
        std::this_thread::sleep_until(time);
    }

    return 0;
}

void decide_all(pi_comm *comm, imu *im, laser *l, controller *cont){
    is = comm->read_data();

    mp.gait = is.gait;
    mp.speed = is.speed;

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

    od.id = im->read_integrated_data();
    
    pl.x = od.id.x.f;
    pl.y = od.id.y.f;
    pl.rz = od.id.rz.f;

    l->write_pos(pl);
    obs = l->read_obs();

    od.os.speed = is.speed;
    od.os.gait = is.gait;
    od.os.obstacle = obs.flag;

    comm->write_od(&od);
}
