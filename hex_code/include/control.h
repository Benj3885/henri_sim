#ifndef CONTROL_H
#define CONTROL_H
#include "control.h"
#include <sstream>
#include <math.h>
#include <mutex>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "rosgraph_msgs/Clock.h"

struct hex_command{
    int i = -1;

    int motor[18] = {0};
    short int pos[18] = {0};
};

struct move_para{
    bool W, S;
    float turn;
};

struct kinematics{
    #define RESOLUTION 10 // Make this a multiple of 10

    float pointPosf[3][5] = {{153.1, 103.5, 103.5, 128.3, 153.1}, {37.7, 87.2, 87.2, 62.5, 37.7}, {-120, -120, -40, 0, -40}};
    float pointPosc[3][5] = {{135, 135, 135, 135, 135}, {-35, 35, 35, 0, -35}, {-120, -120, -40, 0, -40}};
    float pointPosb[3][5] = {{103.5, 153.1, 153.1, 128.3, 103.5}, {0, 49.5, 49.5, 24.7, 0}, {-120, -120, -40, 0, -40}};

    float pointPosfc[3][5] = {{180.5, 130.3, 130.3, 155.1, 180.5}, {-34.9, 14.7, 14.7, -10.1, -34.9}, {-120, -120, -40, 0, -40}};

    float l1 = 53.97, l2 = 80.94, l3 = 120.24;
    float c1, c2, c3, c4;

    float part1rf[3][RESOLUTION], part2rf[3][RESOLUTION];
    float part1lf[3][RESOLUTION], part2lf[3][RESOLUTION];

    float part1rfc[3][RESOLUTION], part2rfc[3][RESOLUTION];
    float part1lfc[3][RESOLUTION], part2lfc[3][RESOLUTION];

    float part1rc[3][RESOLUTION], part2rc[3][RESOLUTION];
    float part1lc[3][RESOLUTION], part2lc[3][RESOLUTION];

    float part1rb[3][RESOLUTION], part2rb[3][RESOLUTION];
    float part1lb[3][RESOLUTION], part2lb[3][RESOLUTION];

    void create_gait(float pointPos[3][5], float part1r[3][RESOLUTION], float part2r[3][RESOLUTION],
                                           float part1l[3][RESOLUTION], float part2l[3][RESOLUTION]);

    float part1rrotc[3][RESOLUTION], part2rrotc[3][RESOLUTION];
    float part1lrotc[3][RESOLUTION], part2lrotc[3][RESOLUTION];

    float part1rrotf[3][RESOLUTION], part2rrotf[3][RESOLUTION];
    float part1lrotf[3][RESOLUTION], part2lrotf[3][RESOLUTION];

    float part1rrotb[3][RESOLUTION], part2rrotb[3][RESOLUTION];
    float part1lrotb[3][RESOLUTION], part2lrotb[3][RESOLUTION];

    float turnPos[2][5] = {{0, M_PI/12, M_PI/12, M_PI/24, 0}, {-120, -120, -40, 0, -40}};

    float off_set_coordc[2] = {-71.4502,     0};
    float off_set_coordf[2] = {-111.1396, -10.0938};
    float off_set_coordb[2] = {-111.1396,  10.0938};

    float rfb = 246.3, rc = 206.4;
    

    kinematics();

    void getIK(float x, float y, float z, float* out);

    void create_turn_gait(float pointPos[2][5], float r, float zero_angle, float frame_angle, float off_set[2],
    float part1r[3][RESOLUTION], float part2r[3][RESOLUTION], float part1l[3][RESOLUTION], float part2l[3][RESOLUTION]);
    float make_range(float in);
};


struct controller{
    std::stringstream ss;
    hex_command hc;
    move_para mp;

    ros::NodeHandle *n;
    ros::Publisher cont[18];
    std_msgs::Float64 mess[18];

    std::mutex *mtx;

    int fd;
    float time_move;
    int speed_max;

    kinematics k;

    //ids and offset
    //RIght side first from front to back
    char servo_id[6][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {9, 10, 11}, {12, 13, 14}, {15, 16, 17}};
    float turnLimit = M_PI/9;

    controller(ros::NodeHandle *mainn);

    void main();

    void tri_gait(bool dir);
    void climb_gait(bool dir);
    void go_to_zero();
    void turn(float turn);
    void control_gait();
    void go_to_gait_start();

    void save_command(char id, float angle);
    void send_command();
    void write_mp(move_para in_mp);
};

#endif