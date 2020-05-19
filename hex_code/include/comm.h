#ifndef COMM_H
#define COMM_H
#include "comm.h"
#include <sys/socket.h>    //socket
#include <arpa/inet.h> //inet_addr
#include <poll.h>
#include <mutex>

#define inBuff 6
#define outBuff 24

struct in_state{
    bool W = 0, A = 0, S = 0, D = 0;
    uint8_t speed = 100;
    uint8_t gait = 1;
};

struct out_state{
    bool W = 0, A = 0, S = 0, D = 0;
    uint8_t speed = 0;
    uint8_t gait = 0;
    bool obstacle = 0;
};

union fval{
    float f = 0;
    char c[4];
};

struct integrated_data{
    fval x, y, z;
    fval vx, vy, vz;
    fval rx, ry, rz;
    fval temp;
};

struct out_data{
    out_state os;
    integrated_data id;
};

struct pi_comm{
    // data
    bool W = 0, A = 0, S = 0, D = 0;
    char speed = 0;

    // TCP knowledge
    int sockfd;
    sockaddr_in server;
    in_state is;
    out_data od;
    std::mutex *read_mtx, *write_mtx;

    char in_mess[inBuff], out_mess[outBuff];

    pi_comm(const char *IP, const int PORT);

    void main();
    void get_data_in();
    void send_data_out();

    in_state read_data();
    void write_od(out_data *od_main);

};

#endif