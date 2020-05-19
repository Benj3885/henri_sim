#include "map_nav.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <mutex>
#include <math.h>
#include "iostream"

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include "sensor_msgs/Image.h"
#include "log_hex/log.h"
#include "cv_bridge/cv_bridge.h"
#include "tf/transform_datatypes.h"
#include <algorithm> // std::find 
#include <ros/callback_queue.h>
#include "geometry_msgs/PoseStamped.h"

void setGeometry(const grid_map::Length& length, const double resolution);

map_nav::map_nav(ros::NodeHandle *mainn1, ros::NodeHandle *mainn2){
    /*pos_mtx = new std::mutex;
    obs_mtx = new std::mutex;*/
    nm = mainn1;
    nn = mainn2;

    std::thread mapthread(&map_nav::map_main, this);
    mapthread.detach();

    std::thread navthread(&map_nav::nav_main, this);
    navthread.detach();
}

void map_nav::map_main(){
    for(int i = 0; i < IM_WIDTH; i++){
        ch[i] = cos((i * H_FOV / (IM_WIDTH - 1)) - 0.5 * H_FOV);
        sh[i] = -sin((i * H_FOV / (IM_WIDTH - 1)) - 0.5 * H_FOV);
    
    }
    float V_FOV = (IM_HEIGHT / (float)IM_WIDTH) * H_FOV;
    for(int i = 0; i < IM_HEIGHT; i++){
        cv[i] = cos((i * V_FOV / (IM_HEIGHT - 1)) - 0.5 * V_FOV);
        sv[i] = -sin((i * V_FOV / (IM_HEIGHT - 1)) - 0.5 * V_FOV);
    }

    mpub = nm->advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);

    map.setFrameId("map");
    // MAP_W x MAP_L map in meters, with a grid size of second input
    map.setGeometry(grid_map::Length(MAP_W, MAP_L), (float)1/GRIDS_PER_METER);
    obsQueue.reserve(1000);

    ros::Subscriber ims = nm->subscribe<log_hex::log>("/hex_log", 1, &map_nav::imcb, this);

    ros::spin();
}


void map_nav::imcb(const log_hex::logConstPtr& msg){
    cv::Mat im = cv_bridge::toCvCopy(msg->im, sensor_msgs::image_encodings::TYPE_32FC1)->image;

    float reading;

    tf::Vector3 p;
    p.setX(msg->pose.position.x);
    p.setY(msg->pose.position.y);
    p.setZ(msg->pose.position.z);

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.orientation, q);
    tf::Transform t(q, tf::Vector3(tfScalar(p.getX()), tfScalar(p.getY()), tfScalar(p.getZ())));

    for(int i = 0; i < IM_WIDTH; i++){
        for(int j = 0; j < IM_HEIGHT; j++){
            reading = im.at<float>(j, i);
            if(reading != reading){
                continue;
            }

            p.setX(ch[i] * (reading * cv[j]) + 0.09);
            p.setY(sh[i] * (reading * cv[j]));
            p.setZ(sv[j] * (reading * ch[i]) + 0.0125);

            p = t.operator()(p);

            if((map.isInside(grid_map::Position(p.getX(), p.getY())) && 
            p.getZ() < 0.5) && 
            (p.getZ() > map.atPosition("elevation", grid_map::Position(p.getX(), p.getY())) ||
            map.atPosition("elevation", grid_map::Position(p.getX(), p.getY())) != map.atPosition("elevation", grid_map::Position(p.getX(), p.getY())))
            ){
                setPos(p.getX(), p.getY(), p.getZ());
            }
        }
    }

    // Publish grid map.
    map.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map, message);
    mpub.publish(message);
}


void map_nav::setPos(float x, float y, float z){
    map.atPosition("elevation", grid_map::Position(x, y)) = z;
    grid_map::Index idx;
    map.getIndex(grid_map::Position(x, y), idx);

    nMap[idx(0)][idx(1)].z = z;

    checkObs(IndexFixed(idx));
}

void map_nav::checkObs(IndexFixed idx){
    IndexFixed iTemp;
    if(idx.x < TOTAL_IDX - 2)
        if(abs(nMap[idx.x][idx.y].z - nMap[idx.x+1][idx.y].z) > 0.05){
            insertObsQueue(idx);
            iTemp = idx;
            iTemp.x++;
            insertObsQueue(iTemp);
        }
    if(idx.x > 0)
        if(abs(nMap[idx.x][idx.y].z - nMap[idx.x-1][idx.y].z) > 0.05){
            insertObsQueue(idx);
            iTemp = idx;
            iTemp.x--;
            insertObsQueue(iTemp);
        }
    if(idx.y < TOTAL_IDX - 2)
        if(abs(nMap[idx.x][idx.y].z - nMap[idx.x][idx.y+1].z) > 0.05){
            insertObsQueue(idx);
            iTemp = idx;
            iTemp.y++;
            insertObsQueue(iTemp);
        }
    if(idx.y > 0)
        if(abs(nMap[idx.x][idx.y].z - nMap[idx.x][idx.y-1].z) > 0.05){
            insertObsQueue(idx);
            iTemp = idx;
            iTemp.y--;
            insertObsQueue(iTemp);
        }
}

void map_nav::insertObsQueue(IndexFixed idx){
    if (std::find(obsQueue.begin(), obsQueue.end(), idx) != obsQueue.end())
        obsQueue.push_back(idx);
}

void map_nav::spreadO(){
    std::vector<IndexFixed> obsVec[2];
    IndexFixed iTemp;
    obsVec[0].reserve((O_VAL - 1) * 4);
    obsVec[1].reserve((O_VAL - 1) * 4);
    bool vIdx = 1;
    unsigned int x, y;

    while(!obsQueue.empty()){
        obsVec[0].push_back(obsQueue.back());
        obsQueue.pop_back();

        for(int i = O_VAL; i > 0; i--){
            vIdx = !vIdx;

            while(!obsVec[vIdx].empty()){
                x = obsVec[vIdx].back().x;
                y = obsVec[vIdx].back().y;
                nMap[x][y].O = i;

                if(x < TOTAL_IDX - 2)
                    if(nMap[x+1][y].O+1 < i){
                        iTemp = obsVec[vIdx].back();
                        iTemp.x++;
                        insertObsVec(obsVec[!vIdx], iTemp);
                    }
                if(x > 0)
                    if(nMap[x-1][y].O+1 < i){
                        iTemp = obsVec[vIdx].back();
                        iTemp.x--;
                        insertObsVec(obsVec[!vIdx], iTemp);
                    }
                if(y < TOTAL_IDX - 2)
                    if(nMap[x][y+1].O+1 < i){
                        iTemp = obsVec[vIdx].back();
                        iTemp.y++;
                        insertObsVec(obsVec[!vIdx], iTemp);
                    }
                if(y > 0)
                    if(nMap[x][y-1].O+1 < i){
                        iTemp = obsVec[vIdx].back();
                        iTemp.y--;
                        insertObsVec(obsVec[!vIdx], iTemp);
                    }

                obsVec[vIdx].pop_back();
            }
        }
    }
}

void map_nav::insertObsVec(std::vector<IndexFixed> &vec, IndexFixed val){
    if (std::find(vec.begin(), vec.end(), val) != vec.end())
        vec.push_back(val);
}


void map_nav::nav_main(){
    ros::CallbackQueue tarQueue;
    nn->setCallbackQueue(&tarQueue);

    ros::Subscriber tarSub = nn->subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &map_nav::tarCB, this);

    while(1){

    }
}

void map_nav::tarCB(const geometry_msgs::PoseStampedConstPtr& msg){

}








