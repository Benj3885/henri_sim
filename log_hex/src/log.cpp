#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/Image.h"
#include "log_hex/log.h"

gazebo_msgs::ModelStates msIn;
sensor_msgs::Image mIn;
log_hex::log l;
ros::Publisher pub;
bool r = 0;

void msCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    r = 1;
    int s = msg->name.size();
    for(int i = 0; i < s; i++){
        if(msg->name[i] == "henry"){
            l.pose = msg->pose[i];
        }
    }
}

void camCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    if(r){
        l.im = *msg;
        pub.publish(l);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gaz_log_node");

    ros::NodeHandle n;

    ros::Subscriber lsSub = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, msCallback);
    ros::Subscriber camSub = n.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 1, camCallback);
    pub = n.advertise<log_hex::log>("/hex_log", 1);

    ros::spin();

    return 0;
}