/****************************
 * Program name: Bombard_terminal_nav.cpp
 * Description: 
 *  This program subscribes current GPS location, alt of the uav and 
 * switches to off-board terminal navigation mode when the UAV enters 
 * a certain range of the bombing bullseye to guide the UAV with 
 * bombardment termianl navgation
 * 
 * Version : 0.1
 * Auther: SHEN CHANG TE
 * Date: 20200723
 * *************************/

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointReached.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <iostream>
#include <math.h>
#include "wgs_conversions/WgsConversion.h"

#define TERMINAL_RADIUS 200  // When enter this circle, switch to off_board\
                            // guidance mode.



// Global variables
mavros_msgs::State current_state;
sensor_msgs::NavSatFix global_position;
static mavros_msgs::WaypointList WP_List; 
mavros_msgs::WaypointReached WP_reached;
bool global_position_received = false;
bool qg_wp_received = false;
bool home_alt_set = false;
std_msgs::Bool bomb_released;
std_msgs::Float64 homeAlt;
using namespace std;

// callback functions
void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    global_position = *msg;
    global_position_received = true;
    ROS_INFO_ONCE("Got global position: [%.2f, %.2f, %.2f]", msg->latitude, msg->longitude, msg->altitude);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void qg_wp_cb(const mavros_msgs::WaypointList::ConstPtr& msg){
    WP_List = *msg;

}

void wp_reached_cb(const mavros_msgs::WaypointReached::ConstPtr& msg){
    WP_reached = *msg;
}

void bomb_status_cb(const std_msgs::Bool::ConstPtr& msg){
    bomb_released = *msg;
}
//----------------------------------------------------

void home_gps_alt_set(void){
    homeAlt.data = global_position.altitude;
    home_alt_set = true;
}

// main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    ros::ServiceClient arming_client = nh.serviceClient < mavros_msgs::CommandBool > ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient < mavros_msgs::SetMode > ("mavros/set_mode");
    ros::Subscriber state_sub = nh.subscribe < mavros_msgs::State > ("mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe < sensor_msgs::NavSatFix > ("mavros/global_position/global", 1, globalPosition_cb);
    ros::Subscriber qg_wp_sub = nh.subscribe <mavros_msgs::WaypointList> ("mavros/mission/waypoints",30, qg_wp_cb);
    ros::Subscriber wp_reached_sub = nh.subscribe <mavros_msgs::WaypointReached> ("mavros/mission/reached",1, wp_reached_cb);
    ros::Subscriber bomb_status_sub = nh.subscribe <std_msgs::Bool> ("bomb_status",1,bomb_status_cb);
    //ros::Publisher goal_pos_pub = nh.advertise < mavros_msgs::GlobalPositionTarget > ("mavros/setpoint_position/global", 10);
    wgs_conversions::WgsConversion srv;
    ros::ServiceClient client = nh.serviceClient<wgs_conversions::WgsConversion>("lla2enu");

    ros::Rate rate(10);

    double curr_lla[3],reference_lla[3],enu[3];
    int WP_interval[2]; // WP_r states for waypoint reached.
    nh.param("bombard_terminal_nav_node/target_lat",reference_lla[0],23.1063974);
    nh.param("bombard_terminal_nav_node/target_long",reference_lla[1],120.2105858);
    nh.param("bombard_terminal_nav_node/target_alt",reference_lla[2],30.0);
    nh.param("bombard_terminal_nav_node/target_pre_wp",WP_interval[0],7);
    nh.param("bombard_terminal_nav_node/target_now_wp",WP_interval[1],8);


    bomb_released.data = false;
    homeAlt.data = 0.0;  // initialize home alt.

    // wait for fcu connection
    while (ros::ok() && !current_state.connected) {
        ROS_INFO_ONCE("Waiting for FCU connection...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    // wait for position information
    /*while (ros::ok() && !global_position_received) {
        ROS_INFO_ONCE("Waiting for GPS signal...");
        ros::spinOnce();
        rate.sleep();
    }*/
    //ROS_INFO("GPS position received");

    if(!home_alt_set){
        home_gps_alt_set();
        printf("home alt is now set to %f", homeAlt.data);
    }

    // Set target waypoint
    mavros_msgs::GlobalPositionTarget goal_position;
    goal_position.latitude = reference_lla[0];
    goal_position.longitude = reference_lla[1];
    goal_position.altitude = reference_lla[2]+homeAlt.data;

    // Set ref_lla for wgs_conversions service
    srv.request.ref_lla[0] = reference_lla[0];
    srv.request.ref_lla[1] = reference_lla[1];
    srv.request.ref_lla[2] = reference_lla[2]+homeAlt.data;

    mavros_msgs::SetMode offb_set_mode, mission_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mission_set_mode.request.custom_mode = "AUTO.MISSION";

    
    float H_DIST;

    // Check the horizontal distance between current and target position
    while (ros::ok()){
        // set current lla for wgs_conversions service
        srv.request.lla[0] = global_position.latitude;
        srv.request.lla[1] = global_position.longitude;
        srv.request.lla[2] = global_position.altitude;

        //goal_pos_pub.publish(goal_position);
        


        if(client.call(srv)){
            enu[0] = srv.response.enu[0];
            enu[1] = srv.response.enu[1];
            enu[2] = srv.response.enu[2];
            H_DIST = sqrt(enu[0]*enu[0]+enu[1]*enu[1]);

            //ROS_INFO("DIST %.2f, E %.2f N %.2f U %.2f", H_DIST, enu[0], enu[1], enu[2]);
            // if horizontal distance between target and UAV < 500m &&
            // bomb is not released && inside the waypoint interval
            if( H_DIST < TERMINAL_RADIUS &&\
            bomb_released.data == false && (int) WP_reached.wp_seq >= WP_interval[0]\
            && (int) WP_List.current_seq <= WP_interval[1]){
                if( current_state.mode != "OFFBOARD" && set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                /*while (!bomb_released.data && current_state.mode == "OFFBOARD"){
                    //ROS_INFO("waiting for bomb release.");
                    ROS_INFO("DIST %.2f, Released %s",H_DIST, bomb_released.data ? "true":"false");
                    //goal_pos_pub.publish(goal_position);
                    rate.sleep();
                }*/

            }
            if (bomb_released.data == true){
                if( current_state.mode != "AUTO.MISSION" && set_mode_client.call(mission_set_mode)&&
                mission_set_mode.response.mode_sent){
                    ROS_INFO("Mission enabled");
                    break;
                }
            }
        }
        ros::spinOnce();
        //ROS_INFO("current_WP %d, DIST %.2f, Released %s",(int) WP_List.current_seq,\
        H_DIST, bomb_released.data ? "true":"false");
        rate.sleep();

    }
    ros::Duration(5.0).sleep();
    while(bomb_released.data){}

    return 0;
}


