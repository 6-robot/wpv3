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
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

static std::string pub_topic;
class CWPRLidarFilter
{
public:
    CWPRLidarFilter();
private:
     ros::NodeHandle n;
     ros::Publisher scan_pub;
     ros::Subscriber scan_sub;
     void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

CWPRLidarFilter::CWPRLidarFilter()
{
    scan_pub = n.advertise<sensor_msgs::LaserScan>(pub_topic,1);
    scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan_raw",1,&CWPRLidarFilter::lidarCallback,this);
}

void CWPRLidarFilter::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //ROS_INFO("[wpv3_lidar_filter]");
    int nRanges = scan->ranges.size();
    sensor_msgs::LaserScan new_scan;
    new_scan.header.stamp = scan->header.stamp;
    new_scan.header.frame_id = scan->header.frame_id;
    new_scan.angle_max = scan->angle_max;
    new_scan.angle_min = scan->angle_min;
    new_scan.angle_increment = scan->angle_increment;
    new_scan.time_increment = scan->time_increment;
    new_scan.range_min = 0.4;
    new_scan.range_max = scan->range_max;
    new_scan.ranges.resize(nRanges);
    new_scan.intensities.resize(nRanges);
    for(int i=0 ; i<nRanges ; i++)
    {
        new_scan.ranges[i] = scan->ranges[i];
        new_scan.intensities[i] = scan->intensities[i];
    }
    // 剔除货架腿遮挡部分
    int nRemoveAngle = 25;      //遮挡的角度跨度
    int nLeftStartIndex = 160;  //开始遮挡的线数

    int nRightStartIndex = 1081 - nLeftStartIndex;
    for(int i=nLeftStartIndex ; i<nLeftStartIndex+nRemoveAngle*4 ; i++)
    {
        new_scan.ranges[i] = new_scan.range_max+1.0;
        new_scan.intensities[i] = 0;
    }
    for(int i=nRightStartIndex ; i>nRightStartIndex-nRemoveAngle*4 ; i--)
    {
        new_scan.ranges[i] = new_scan.range_max+1.0;
        new_scan.intensities[i] = 0;
    }
    scan_pub.publish(new_scan);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpv3_lidar_filter");

    ros::NodeHandle n_param("~");

    n_param.param<std::string>("pub_topic", pub_topic, "/scan_filtered");

    CWPRLidarFilter lidar_filter;
    ros::spin();
}