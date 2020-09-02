/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "kl_outlier.h"

static std::string pc_topic;
static ros::Publisher vel_pub;
static ros::Publisher pc_pub;
static ros::Publisher marker_pub;
static bool bActive = false;
static float ranges[1081];
static float keep_dist = 1.0;   //跟随距离
static float flw_x = keep_dist;
static float flw_y = 0;
static float new_flw_x = flw_x;
static float new_flw_y = flw_y;
static float max_linear_vel = 1.5;
static float max_angular_vel = 2.0;

void ScanCB(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    for(int i=0;i<1081;i++)
    {
        ranges[i] = scan->ranges[i];
    }

    filter(flw_x,flw_y,ranges,ranges,new_flw_x,new_flw_y);
    flw_x = new_flw_x;
    flw_y = new_flw_y;

    geometry_msgs::Twist vel_cmd;
    float flw_dist = sqrt(flw_x*flw_x + flw_y*flw_y);
    float diff_dist = flw_dist - keep_dist;
    float flw_linear = diff_dist * 0.5;
    if(fabs(flw_linear) > 0.05)
    {
        vel_cmd.linear.x = flw_linear;
        if( vel_cmd.linear.x > max_linear_vel ) vel_cmd.linear.x = max_linear_vel;
        if( vel_cmd.linear.x < -max_linear_vel ) vel_cmd.linear.x = -max_linear_vel;
        if( vel_cmd.linear.x < 0 ) vel_cmd.linear.x *= 0.3;
    }
    else
    {
        vel_cmd.linear.x = 0;
    }
    float d_angle = 0;
    float abs_x = fabs(new_flw_x);
    if(abs_x != 0) d_angle = atan(flw_y/abs_x);
    float flw_turn = d_angle * 2;
    if(fabs(flw_turn) > 0.1)
    {
        vel_cmd.angular.z = flw_turn;
        if( vel_cmd.angular.z > max_angular_vel ) vel_cmd.angular.z = max_angular_vel;
        if( vel_cmd.angular.z < -max_angular_vel ) vel_cmd.angular.z = -max_angular_vel;
    }
    else
    {
        vel_cmd.angular.z = 0;
    }
    if(bActive == true)
    {
        vel_pub.publish(vel_cmd);
    }
}

void BehaviorCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("follow start");
    if( nFindIndex >= 0 )
    {
        flw_x = 0.7;
        flw_y = 0;
        keep_dist = flw_x;
        thredhold(flw_x);
        bActive = true;
        ROS_WARN("[follow_start] flw_x = %.2f", flw_x);
    }

    nFindIndex = msg->data.find("follow stop");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[follow_stop] ");
        bActive = false;
        geometry_msgs::Twist vel_cmd;
        vel_cmd.linear.x = 0;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = 0;
        vel_pub.publish(vel_cmd);
    }

    nFindIndex = msg->data.find("follow resume");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[follow_resume]");
        bActive = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpv3_follow_server");
    ROS_WARN("[wpv3_follow_server]");

    flw_x = keep_dist;
    flw_y = 0;
    thredhold(flw_x);

    ros::NodeHandle nh_param("~");
    nh_param.param<bool>("start", bActive, false);

    ros::NodeHandle nh;
    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan",30,ScanCB);

    ros::Subscriber sub_sr = nh.subscribe("/wpv3/behaviors", 30, BehaviorCB);

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::spin();

    return 0;
}