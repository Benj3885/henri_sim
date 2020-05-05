#include "comm.h"
#include <sys/socket.h>    //socket
#include <arpa/inet.h> //inet_addr
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>    //write
#include <poll.h>
#include <chrono>
#include <thread>
#include <mutex>
#include "motion.h"

pi_comm::pi_comm(const char *IP, const int PORT){
    read_mtx = new std::mutex();
    write_mtx = new std::mutex();

    server.sin_addr.s_addr = inet_addr(IP);
    server.sin_family = AF_INET;
    server.sin_port = htons(PORT);

    sockfd = socket(AF_INET, SOCK_STREAM , 0);

    if(connect(sockfd, (struct sockaddr *) &server, sizeof(server)) < 0)
    {
        printf("Connection to laptop failed\n");
        exit(0);
    }

    std::thread commthread(&pi_comm::main, this);
    commthread.detach();
}

void pi_comm::main(){
    int interval = 20;
    auto time_interval = std::chrono::milliseconds(interval);
    auto time = std::chrono::steady_clock::now();
    
    while(1){
        time += time_interval;
        send_data_out();
        get_data_in();
        std::this_thread::sleep_until(time);
    }
}

void pi_comm::get_data_in(){
    recv(sockfd, in_mess, inBuff, 0);
    
    read_mtx->lock();

    is.W = in_mess[0];
    is.A = in_mess[1];
    is.S = in_mess[2];
    is.D = in_mess[3];
    is.speed = in_mess[4];
    is.gait = in_mess[5];

    read_mtx->unlock();
}

void pi_comm::send_data_out(){
    write_mtx->try_lock();

    out_mess[0] = od.os.W;
    out_mess[1] = od.os.A;
    out_mess[2] = od.os.S;
    out_mess[3] = od.os.D;
    out_mess[4] = od.os.speed;
    out_mess[5] = od.os.gait;
    out_mess[6] = od.os.obstacle;

    int idx = 7;
    for(int i = 0; i < 4; i++){
        out_mess[idx] = od.id.x.c[i]; idx++;
    }
    for(int i = 0; i < 4; i++){
        out_mess[idx] = od.id.y.c[i]; idx++;
    }
    for(int i = 0; i < 4; i++){
        out_mess[idx] = od.id.rz.c[i]; idx++;
    }
    for(int i = 0; i < 4; i++){
        out_mess[idx] = od.id.temp.c[i]; idx++;
    }

    write_mtx->unlock();

    write(sockfd, out_mess, outBuff);
}

in_state pi_comm::read_data(){
    std::lock_guard<std::mutex> lock(*read_mtx);
    return is;
}

void pi_comm::write_od(out_data *od_main){
    std::lock_guard<std::mutex> lock(*write_mtx);
    od = *od_main;
}