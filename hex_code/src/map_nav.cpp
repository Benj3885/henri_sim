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

    mtx = new std::mutex;

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
    mtx->lock();
    rs.x = msg->pose.position.x;
    rs.y = msg->pose.position.y;
    rs.rz = std::atan2(
        2 * (msg->pose.orientation.w * msg->pose.orientation.z + msg->pose.orientation.x * msg->pose.orientation.y),
        1 - 2 * (msg->pose.orientation.y * msg->pose.orientation.y + msg->pose.orientation.z * msg->pose.orientation.z)
    );
    mtx->unlock();

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
    nMap[idx(0)][idx(1)].mapped = 1;

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
    if (std::find(obsQueue.begin(), obsQueue.end(), idx) == obsQueue.end())
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
    if (std::find(vec.begin(), vec.end(), val) == vec.end())
        vec.push_back(val);
}


void map_nav::nav_main(){
    tarQueue = new ros::CallbackQueue();
    nn->setCallbackQueue(tarQueue);

    activeNeurons.reserve(10000);
    pendingNeurons.reserve(1000);

    ros::Subscriber tarSub = nn->subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &map_nav::tarCB, this);

    while(1){
        tarQueue->callAvailable(ros::WallDuration(0.1));
    }
}

void map_nav::tarCB(const geometry_msgs::PoseStampedConstPtr& msg){
    rs.gx = msg->pose.position.x;
    rs.gy = msg->pose.position.y;
    rs.grz = std::atan2(
        2 * (msg->pose.orientation.w * msg->pose.orientation.z + msg->pose.orientation.x * msg->pose.orientation.y),
        1 - 2 * (msg->pose.orientation.y * msg->pose.orientation.y + msg->pose.orientation.z * msg->pose.orientation.z)
    );

    grid_map::Index target;

    map.getIndex(grid_map::Position(rs.gx, rs.gy), target);
    rs.gxg = target(0);
    rs.gyg = target(1);

    map.getIndex(grid_map::Position(rs.x, rs.y), target);
    rs.xg = target(0);
    rs.yg = target(1);

    DWENN();
}

void map_nav::DWENN(){
    if(!ins(rs.gxg, rs.gyg) || !nMap[rs.gxg][rs.gyg].mapped || nMap[rs.gxg][rs.gyg].O){
        std::cout << "Target not in allowed space, seek permission from space" << std::endl;
        return;
    }

    nMap[rs.gxg][rs.gyg].val = 1;
    nMap[rs.gxg][rs.gyg].stage = 2;

    activateNeighbourNeurons();

    int x, y;

    x = rs.xg;
    y = rs.yg;

    while(rs.xg != rs.gxg && rs.yg != rs.gyg){

        while(nMap[rs.xg][rs.yg].stage != 2 || x != rs.xg || y != rs.yg){
            if(!tarQueue->empty()){
                resetNMap();
                return;
            }

            updateActiveNeurons();

            if(nMap[rs.gxg][rs.gyg].stage == 0 && pendingNeurons.empty()){
                std::cout << "404: path not found" << std::endl;
            }

            updatePendingNeurons();
        }

        // INSERT MOVEMENT STUFF

    }
}

void map_nav::updateActiveNeurons(){
    int x, y;

    for(int i = 0; i < activeNeurons.size(); i++){
        x = activeNeurons[i].x;
        y = activeNeurons[i].y;
        if(nMap[x][y].O){
            nMap[x][y].deactivate();
            activeNeurons.erase(activeNeurons.begin()+i);
            continue;
        }
        if(nMap[x][y].wp->val){
            findPend(IndexFixed(x, y));
            nMap[x][y].val++;
            continue;
        }
        if(!findActiveNeighbour(x, y)){
            nMap[x][y].deactivate();
            activeNeurons.erase(activeNeurons.begin()+i);
            continue;
        }
        findPend(IndexFixed(x, y));
    }
}

void map_nav::updatePendingNeurons(){
    int x, y;

    while(!pendingNeurons.empty()){
        x = pendingNeurons.back().x;
        y = pendingNeurons.back().y;
        
        if(nMap[x][y].formerVal){
            nMap[x][y].formerVal = 0;
            nMap[x][y].stage = 0;
            pendingNeurons.pop_back();
            continue;
        }
        if(!findActiveNeighbour(x, y)){
            nMap[x][y].stage = 0;
        }

        pendingNeurons.pop_back();
    }
}

bool map_nav::findActiveNeighbour(int x, int y){
    if(ins(x+1, y)){
        if(!nMap[x+1][y].O && nMap[x+1][y].stage == 2 && nMap[x+1][y].val < nMap[x][y].val){
            nMap[x][y].activate(nMap[x+1][y].val + 1, 3, &nMap[x+1][y]);
            return 1;
        }
    }
    if(ins(x-1, y)){
        if(!nMap[x-1][y].O && !nMap[x-1][y].stage == 2 && nMap[x-1][y].val < nMap[x][y].val){
            nMap[x][y].activate(nMap[x-1][y].val + 1, 1, &nMap[x-1][y]);
            return 1;
        }
    }
    if(ins(x, y+1)){
        if(!nMap[x][y+1].O && !nMap[x][y+1].stage == 2 && nMap[x][y+1].val < nMap[x][y].val){
            nMap[x][y].activate(nMap[x][y+1].val + 1, 4, &nMap[x][y+1]);
            return 1;
        }
    }
    if(ins(x, y-1)){
        if(!nMap[x][y-1].O && !nMap[x][y-1].stage == 2 && nMap[x][y-1].val < nMap[x][y].val){
            nMap[x][y].activate(nMap[x][y-1].val + 1, 2, &nMap[x][y-1]);
            return 1;
        }
    }

    return 0;
}

void map_nav::activateNeighbourNeurons(){
    IndexFixed temp;
    if(ins(rs.gxg+1, rs.gyg)){
        if(!nMap[rs.gxg+1][rs.gyg].O && !nMap[rs.gxg+1][rs.gyg].stage && nMap[rs.gxg+1][rs.gyg].mapped){
            nMap[rs.gxg+1][rs.gyg].activate(1, 3, &nMap[rs.gxg][rs.gyg]);
            temp.write(rs.gxg+1, rs.gyg);
            activeNeurons.push_back(temp);
            findPend(temp);
        }
    }
    if(ins(rs.gxg-1, rs.gyg)){
        if(!nMap[rs.gxg-1][rs.gyg].O && !nMap[rs.gxg-1][rs.gyg].stage && nMap[rs.gxg-1][rs.gyg].mapped){
            nMap[rs.gxg-1][rs.gyg].activate(1, 1, &nMap[rs.gxg][rs.gyg]);
            temp.write(rs.gxg-1, rs.gyg);
            activeNeurons.push_back(temp);
            findPend(temp);
        }
    }
    if(ins(rs.gxg, rs.gyg+1)){
        if(!nMap[rs.gxg][rs.gyg+1].O && !nMap[rs.gxg][rs.gyg+1].stage && nMap[rs.gxg][rs.gyg+1].mapped){
            nMap[rs.gxg][rs.gyg+1].activate(1, 4, &nMap[rs.gxg][rs.gyg]);
            temp.write(rs.gxg, rs.gyg+1);
            activeNeurons.push_back(temp);
            findPend(temp);
        }
    }
    if(ins(rs.gxg, rs.gyg-1)){
        if(!nMap[rs.gxg][rs.gyg-1].O && !nMap[rs.gxg][rs.gyg-1].stage && nMap[rs.gxg][rs.gyg-1].mapped){
            nMap[rs.gxg][rs.gyg-1].activate(1, 2, &nMap[rs.gxg][rs.gyg]);
            temp.write(rs.gxg, rs.gyg-1);
            activeNeurons.push_back(temp);
            findPend(temp);
        }
    }
}

void map_nav::findPend(IndexFixed idx){
    IndexFixed temp;
    if(ins(idx.x+1, idx.y)){
        if(!nMap[idx.x+1][idx.y].O && !nMap[idx.x+1][idx.y].stage && nMap[idx.x+1][idx.y].mapped){
            temp = idx;
            temp.x++;
            pendingNeurons.push_back(temp);
            nMap[idx.x+1][idx.y].stage = 1;
        }
    }
    if(ins(idx.x-1, idx.y)){
        if(!nMap[idx.x-1][idx.y].O && !nMap[idx.x-1][idx.y].stage && nMap[idx.x-1][idx.y].mapped){
            temp = idx;
            temp.x--;
            pendingNeurons.push_back(temp);
            nMap[idx.x-1][idx.y].stage = 1;
        }
    }
    if(ins(idx.x, idx.y+1)){
        if(!nMap[idx.x][idx.y+1].O && !nMap[idx.x][idx.y+1].stage && nMap[idx.x][idx.y+1].mapped){
            temp = idx;
            temp.y++;
            pendingNeurons.push_back(temp);
            nMap[idx.x][idx.y+1].stage = 1;
        }
    }
    if(ins(idx.x, idx.y-1)){
        if(!nMap[idx.x][idx.y-1].O && !nMap[idx.x][idx.y-1].stage && nMap[idx.x][idx.y-1].mapped){
            temp = idx;
            temp.y--;
            pendingNeurons.push_back(temp);
            nMap[idx.x][idx.y-1].stage = 1;
        }
    }
}

bool map_nav::ins(int x, int y){
    return (x >= 0 && x < TOTAL_IDX && y >= 0 && y < TOTAL_IDX);
}

void map_nav::resetNMap(){
    unsigned int x, y;
    while(!activeNeurons.empty()){
        x = activeNeurons.back().x;
        y = activeNeurons.back().y;

        nMap[x][y].val = 0;
        nMap[x][y].formerVal = 0;
        nMap[x][y].w = 0;
        nMap[x][y].wp = NULL;
        nMap[x][y].stage = 0;
        activeNeurons.pop_back();
    }

    while(!pendingNeurons.empty()){
        x = pendingNeurons.back().x;
        y = pendingNeurons.back().y;

        nMap[x][y].formerVal = 0;
        nMap[x][y].stage = 0;
        pendingNeurons.pop_back();
    }
}

