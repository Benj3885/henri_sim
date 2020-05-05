#include "laser.h"
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <unistd.h>    //write
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <math.h>

// 5 down, 6 to the right

#define IP "169.254.74.191"
#define PORT 2112

laser::laser(){
    pos_mtx = new std::mutex;
    obs_mtx = new std::mutex;

    //Create socket
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock < 0){
        printf("Socket bind failed\n");
        exit(0);
    }
     
    server.sin_addr.s_addr = inet_addr(IP);
    server.sin_family = AF_INET;
    server.sin_port = htons(PORT);
 
    /*//Connect to remote server
    if(connect(sock, (struct sockaddr *) &server, sizeof(server)) < 0){
        printf("Connection to lidar failed\n");
        exit(0);
    }

    std::thread laserthread(&laser::main, this);
    laserthread.detach();*/
}

 
void laser::main()
{    
    start_scan();
    calc_init_values(250);

    recv(sock, tcp_data, 233, 0);

    int interval = 20;
    auto time_interval = std::chrono::milliseconds(interval);
    auto time = std::chrono::steady_clock::now();

    while(1){
        time += time_interval;
        recv(sock, tcp_data, 233, 0);
        read_data();
        sim_obs_det();

        /*for(int i = 0; i < 4; i++){
            for(int j = 0; j < 15; j++){
                printf("%12d   ", dist[i*15 + j]);
            } printf("\n");
        }
        printf("\n\n");*/

        std::this_thread::sleep_until(time);
    }

}

void laser::start_scan(){
    char out[] = {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x11, 0x73, 0x45, 0x4E, 0x20, 
    0x4C, 0x4D, 0x44, 0x73, 0x63, 0x61, 0x6E, 0x64, 0x61, 0x74, 0x61, 0x20, 0x01, 0x33}; 
    
    write(sock, out, sizeof(out) / sizeof(out[0]));
}

void laser::stop_scan(){
    char out[] = {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x11, 0x73, 0x45, 0x4E, 0x20, 
    0x4C, 0x4D, 0x44, 0x73, 0x63, 0x61, 0x6E, 0x64, 0x61, 0x74, 0x61, 0x20, 0x00, 0x33}; 

    write(sock, out, sizeof(out) / sizeof(out[0]));
}

void laser::read_data(){
    int data_start = 85;

    for(int i = 0; i < LIDAR_RANGE; i++){
        dist[i] = (tcp_data[data_start + i*2] << 8) + tcp_data[data_start + i*2+1];
    }
}


void laser::write_pos(pos_lidar pos){
    std::lock_guard<std::mutex> lock(*pos_mtx);
    pl = pos;
}

void laser::calc_init_values(int width){
    double degtorad = 180.0 / M_PI;

    dist_cos_tilt = cos((90 - 9.94) * degtorad);
    dist_sin_tilt = sin((90 - 9.94) * degtorad);

    for(int i = 0; i < LIDAR_RANGE; i++){
        dist_discard[i] = width / sin((i - 30) * degtorad);
        dist_cos_angles[i] = cos((i - 30) * degtorad);
        dist_sin_angles[i] = sin((i - 30) * degtorad);
        dist_sim_const[i] = dist_sin_tilt * dist_cos_angles[i];
        dist_discard_no_obs[i] = 214 / (dist_cos_angles[i] * dist_cos_tilt);
    }
}

void laser::shiftMap(){
    mapIdx = !mapIdx;

    static float formerX = 0, formerY = 0;

    pos_mtx->lock();

    float cz = cos(-pl.rz);
    float sz = sin(-pl.rz);

    float x = formerX - pl.x;
    float y = formerY - pl.y;

    pos_mtx->unlock();

    for(int w = 0; w < MAP_W; w++){
        for(int h = 0; h < MAP_H; h++){
            map[mapIdx][w][h].set = 0;
        }
    }

    for(int w = 0; w < MAP_W; w++){
        for(int h = 0; h < MAP_H; h++){
            if(map[!mapIdx][w][h].set){
                transformPoint(w, h, cz, sz, x, y);
            }
        }
    }
}

void laser::transformPoint(int w, int h, float cz, float sz, float x, float y){
    float tx, ty;
    int ix, iy;

    tx = map[!mapIdx][w][h].x0 * cz - map[!mapIdx][w][h].y0 * sz + x;
    ty = map[!mapIdx][w][h].x0 * sz + map[!mapIdx][w][h].y0 * cz + y;

    if(tx > -250 && tx < 250 && ty < 1500 && ty > 0){
        ix = (int)((tx + 25) / 50);
        iy = (int)((ty + (ty > 0 ? 25 : -25)) / 50) + 25;

        map[mapIdx][ix][iy].write(tx, ty, map[mapIdx][w][h].z0);        
    }

    if(map[mapIdx][w][h].dz != 0){
        tx = map[!mapIdx][w][h].x1 * cz - map[!mapIdx][w][h].y1 * sz + x;
        ty = map[!mapIdx][w][h].x1 * sz + map[!mapIdx][w][h].y1 * cz + y;

        if(tx > -250 && tx < 250 && ty < 1500 && ty > 0){
            ix = (int)((tx + 25) / 50);
            iy = (int)((ty + (ty > 0 ? 25 : -25)) / 50) + 5;

            map[mapIdx][ix][iy].write(tx, ty, map[mapIdx][w][h].z1);        
        }
    }
}

void point::write(float x, float y, float z){
    if(!set){
        set = 1;
        x0 = x1 = x;
        y0 = y1 = y;
        z0 = z1 = z;
        dz = 0;
    } else {
        if(z < z0){
            x0 = x;
            y0 = y;
            z0 = z;
            dz = z1 - z0;
            return; 
        }
        if(z > z1){
            x1 = x;
            y1 = y;
            z1 = z;
            dz = z1 - z0;
            return; 
        }
    }
}

void laser::add_data_to_map(){
    int ix, iy;
    float x, y, z;

    for(int i = 0; i < LIDAR_RANGE; i++){
        if(dist[i] > dist_discard[i]){
            continue;
        }

        x = dist[i] * dist_sin_tilt * dist_cos_angles[i];
        y = dist[i] * dist_sin_tilt * dist_sin_angles[i];
        z = dist[i] * dist_cos_tilt - Lh;

        ix = (int)((x + 25) / 50);
        iy = (int)((y + (y > 0 ? 25 : -25)) / 50) + 5;

        map[mapIdx][ix][iy].write(x, y, z);
    }

}

void laser::check_obstacle(){
    obs.flag = 0;

    for(int i = 0; i < MAP_W; i++){
        scan_col(i);
    }
}

void laser::scan_col(int c){
    /*float accZ = 0;

    float lowZ = 0, highZ = 0;
    float diffZThres = 2.5;

    int pointCount = 0;
    int points[MAP_H];

    for(int i = 0; i < MAP_H; i++){
        if(map[mapIdx][i][c].set){
            points[pointCount] = i;
            pointCount++;
        }
    }


    for(int i = 0; i < pointCount; i++){
        if(!map[mapIdx][points[i]][c].dz){
            lowZ = map[mapIdx][points[i]][c].z0;
            
        }
        
    }*/
}

void laser::sim_obs_det(){
    obs_mtx->lock();

    obs.flag = 0;
    obs.length = 0;
    float length;

    for(int i = 0; i < LIDAR_RANGE; i++){
        if(dist[i] < dist_discard[i] && dist[i] < dist_discard_no_obs[i]){
            length = dist[i] * dist_sim_const[i];
            if(length > obs.length){
                obs.length = length;
            }
        }
    }

    obs_mtx->unlock();
}

obstacle laser::read_obs(){
    std::lock_guard<std::mutex> lock(*obs_mtx);
    return obs;
}




/*void laser::find_obstacle(){
    obsDist = 0;

    int oldIdx = (buffIdx + 1 < OBS_BUFF ? buffIdx + 1 : 0);

    for(int i = 0; i < LIDAR_RANGE; i++){
        if(dist[i][buffIdx] > dist_discard[i]){
            continue;
        }

        if((dist[i][buffIdx] - dist[i][oldIdx]) > 50){
            
        }
    }

}*/

