#ifndef LASER_H
#define LASER_H
#include "map_nav.h"
#include <mutex>
#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "sensor_msgs/Image.h"
#include "log_hex/log.h"
#include "vector"
#include "control.h"
#include "geometry_msgs/PoseStamped.h"
#include <ros/callback_queue.h>

#define IM_WIDTH 100
#define IM_HEIGHT 80
#define H_FOV 1.57079632679
#define MAP_L 10
#define MAP_W 10
#define GRIDS_PER_METER 20
#define TOTAL_IDX MAP_L * GRIDS_PER_METER
#define O_VAL 8

struct neuron{
    float z = NAN;
    unsigned int val = 0;
    char stage = 0;
    bool formerVal = 0;
    int O = 0;
    char w = 0;
    neuron *wp = NULL;
    bool mapped = 0;

    void activate(unsigned int valIn, char wIn, neuron *wpIn){
        val = valIn;
        stage = 2;
        formerVal = 1;
        w = wIn;
        wp = wpIn;
    }

    void deactivate(){
        val = 0;
        stage = 0;
        wp = NULL;
        w = 0;
    }
};

struct IndexFixed{
    int x, y;

    IndexFixed(grid_map::Index idx){
        x = idx(0);
        y = idx(1);
    }

    IndexFixed(){
        x = 0;
        y = 0;
    }

    IndexFixed(int xIn, int yIn){
        x = xIn;
        y = yIn;
    }

    bool operator==(IndexFixed idx) {
	    return x == idx.x && y == idx.y;
    }

    void write(int xIn, int yIn){
        x = xIn;
        y = yIn;
    }
};

struct robotState{
    float x, y, rz;
    int xg, yg;
    move_para mp;
    float gx, gy, grz;
    int gxg, gyg;
};

struct map_nav{
    std::mutex *mtx;
    ros::NodeHandle *nm;
    ros::Publisher mpub;

    // MAP
    grid_map::GridMap map = grid_map::GridMap({"elevation"});
    neuron nMap[MAP_L*GRIDS_PER_METER][MAP_W*GRIDS_PER_METER];
    std::vector<IndexFixed> obsQueue;

    float ch[IM_WIDTH];
    float sh[IM_WIDTH];
    float cv[IM_HEIGHT];
    float sv[IM_HEIGHT];

    map_nav(ros::NodeHandle *mainn1, ros::NodeHandle *mainn2);
    void map_main();

    void imcb(const log_hex::logConstPtr& msg);
    void setPos(float x, float y, float z);
    void checkObs(IndexFixed idx);
    void insertObsQueue(IndexFixed idx);
    void spreadO();
    void insertObsVec(std::vector<IndexFixed> &vec, IndexFixed val);

    robotState rs;
    ros::NodeHandle *nn;
    std::vector<IndexFixed> activeNeurons;
    std::vector<IndexFixed> pendingNeurons;
    ros::CallbackQueue *tarQueue;

    void nav_main();
    void tarCB(const geometry_msgs::PoseStampedConstPtr& msg);
    void DWENN();
    
    
    void updateActiveNeurons();
    void updatePendingNeurons();
    void activateNeighbourNeurons();
    void resetNMap();

    bool ins(int x, int y);
    void checkPend(IndexFixed idx);
    void findPend(IndexFixed idx);
    bool findActiveNeighbour(int x, int y);
};



#endif