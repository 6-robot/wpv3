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

#include <wpv3_local_planner/wpv3_local_planner.h>
#include <tf_conversions/tf_eigen.h>
#include <pluginlib/class_list_macros.h>

// register this planner as a Wpv3LocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(wpv3_local_planner, Wpv3LocalPlanner, wpv3_local_planner::Wpv3LocalPlanner, nav_core::BaseLocalPlanner)

namespace wpv3_local_planner
{
    Wpv3LocalPlanner::Wpv3LocalPlanner()
    {
        //ROS_WARN("[WPV3]Wpv3LocalPlanner() ");
        m_costmap_ros = NULL;
        m_tf_listener = NULL; 
        m_goal_reached = false;
        m_bInitialized = false;
        m_bAC = false;
        m_bFirstStep = true;

        m_pi = 3.1415926;
        pntx = new float[1081];
        pnty = new float[1081];
        sink = new double[1081];
        cosk = new double[1081];
        double kStep = (m_pi*1.5) / 1080;
        for(int i=0;i<1081;i++)
        {
            sink[i] = sin(m_pi*0.75 - kStep*i);
            cosk[i] = cos(m_pi*0.75 - kStep*i);
        }
        for (int i=0;i<1081;i++)
        {
            pntx[i] = 100 * sink[i];
            pnty[i] = 100 * cosk[i];
        }
        ac_y = 0;
        ac_x = 0;
    }

    Wpv3LocalPlanner::~Wpv3LocalPlanner()
    {
        delete []pntx;
        delete []pnty;
        delete []sink;
        delete []cosk;
    }

    void Wpv3LocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("Wpv3LocalPlanner::initialize() ");
        if(!m_bInitialized)
        {	
            m_tf_listener = tf;
            m_costmap_ros = costmap_ros;
            m_costmap = m_costmap_ros->getCostmap();
            
            m_global_frame_id = m_costmap_ros->getGlobalFrameID();      //"odom"
            m_robot_base_frame_id = m_costmap_ros->getBaseFrameID();    //"base_footprint"
            
            m_footprint_spec = m_costmap_ros->getRobotFootprint();
            costmap_2d::calculateMinAndMaxDistances(m_footprint_spec, m_robot_inscribed_radius, m_robot_circumscribed_radius); 

            ros::NodeHandle nh_planner("~/" + name);
            nh_planner.param("max_vel_trans", m_max_vel_trans, 0.3);
            nh_planner.param("max_vel_rot", m_max_vel_rot, 0.9);
            nh_planner.param("acc_scale_trans", m_acc_scale_trans, 0.8);
            nh_planner.param("acc_scale_rot", m_acc_scale_rot, 0.3);
            nh_planner.param("goal_dist_tolerance", m_goal_dist_tolerance, 0.2);
            nh_planner.param("goal_yaw_tolerance", m_goal_yaw_tolerance, 0.2);
            nh_planner.param("scan_topic", m_scan_topic, std::string("/scan"));

            m_pub_target = nh_planner.advertise<geometry_msgs::PoseStamped>("local_planner_target", 10);
            m_scan_sub = nh_planner.subscribe<sensor_msgs::LaserScan>(m_scan_topic,1,&Wpv3LocalPlanner::lidarCallback,this);
            
            m_bInitialized = true;

            ROS_DEBUG("wpv3_local_planner plugin initialized.");
        }
        else
        {
            ROS_WARN("wpv3_local_planner has already been initialized, doing nothing.");
        }
    }

    static float ValLimit(float inVal, float inMin, float inMax)
    {
        float retVal = inVal;
        if(retVal < inMin)
        {
            retVal = inMin;
        }
        if(retVal > inMax)
        {
            retVal = inMax;
        }
        return retVal;
    }
    static float GetMinVal(float inVal1, float inVal2)
    {
        if(inVal1 < inVal2)
        {
            return inVal1;
        }
        else
        {
            return inVal2;
        }
    }

    static double fScaleD2R = 3.14159 / 180;
    void Wpv3LocalPlanner::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        //ROS_INFO("Wpv3LocalPlanner::lidarCallback");
        int nRanges = scan->ranges.size();
        float range_max = scan->range_max;
        float range_min = scan->range_min;
        float ac_width = 0.3;  //碰撞检测的宽度
        float ac_dist = 0.45;   //碰撞检测的距离
        float left_width = ac_width;
        float left_dist = ac_dist;
        float right_width = -ac_width;
        float right_dist = ac_dist;

        //计算激光距离点
        for (int i=0;i<1081;i++)
        {
            float fVal = scan->ranges[i];
            pntx[i] = fVal * cosk[i];
            pnty[i] = fVal * sink[i] * -1;
            //printf("[%d] ( %.2f , %.2f )\n",i,pntx[i],pnty[i]);
        }

        //检测障碍物的距离
        for (int i = 80; i < 1001; i++)
        {
            if(scan->ranges[i] < range_min || scan->ranges[i] > range_max)
                continue;
            if(pnty[i] > 0) //左侧的点
            {
                if(pnty[i] < ac_width && pntx[i] < ac_dist) //在碰撞检测范围内
                {
                    if(pnty[i] < left_width)
                    {
                        left_width = pnty[i];
                    }
                    if(pntx[i] < left_dist)
                    {
                        left_dist = pntx[i];
                    }
                }
            }
            else
            {
                //右侧的点
                if((-1*pnty[i]) < ac_width && pntx[i] < ac_dist) //在碰撞检测范围内
                {
                    if(pnty[i] > right_width)
                    {
                        right_width = pnty[i];
                    }
                    if(pntx[i] < right_dist)
                    {
                        right_dist = pntx[i];
                    }
                }
            }

        }
        //ROS_WARN("left d= %.2f w= %.2f | right d= %.2f w=%.2f",left_dist,left_width,right_dist,right_width);

        // ajust
        if(right_width < ac_width || left_width < ac_width || right_dist < ac_dist || left_dist < ac_dist)
        {
            ac_x = GetMinVal(right_dist,left_dist) - ac_dist;
            ac_y = left_width + right_width;
            if(ac_y < 0.01 && left_width < 0.01 && right_width > -0.01)
            {
                if(left_dist > right_dist)
                {
                    ac_y = 0.6;
                }
                else
                {
                    ac_y = -0.6;
                }
            }
        }
        else
        {
            ac_x = 0;
            ac_y = 0;
        }
        
        ac_x *= 0.8;
        ac_y *= 0.3;
        ac_x = ValLimit(ac_x,-0.2,0.2);
        ac_y = ValLimit(ac_y,-0.2,0.2);
        //ROS_WARN("ac_x = %.2f  ac_y = %.2f",ac_x,ac_y);
    }

    bool Wpv3LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        // ROS_WARN("[WPV3]setPlan() ");
        if(!m_bInitialized)
        {
            ROS_ERROR("wpv3_local_planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        m_global_plan.clear();
        m_global_plan = plan;
        m_nPathIndex = 0;

        m_goal_reached = false;
        m_nStep = WPV3_STEP_GOTO;
        m_bFirstStep = true;
        
        return true;
    }

    static double CalDirectAngle(double inFromX, double inFromY, double inToX, double inToY)
    {
        double res = 0;
        double dx = inFromX - inToX;
        double dy = -(inFromY - inToY);
        if (dx == 0)
        {
            if (dy > 0)
            {
                res = 180 - 90;
            }
            else
            {
                res = 0 - 90;
            }
        }
        else
        {
            double fTan = dy / dx;
            res = atan(fTan) * 180 / 3.1415926;

            if (dx < 0)
            {
                res = res - 180;
            }
        }
        res = 180 - res;
        if (res < 0)
        {
            res += 360;
        }
        if (res > 360)
        {
            res -= 360;
        }
        res = res*3.1415926/180;
        return res;
    }

    static double AngleFix(double inAngle, double inMin, double inMax)
    {
        if (inMax - inMin > 6.28)
        {
            return inAngle;
        }
        
        double retAngle = inAngle;
        while (retAngle < inMin)
        {
            retAngle += 6.28;
        }
        while (retAngle > inMax)
        {
            retAngle -= 6.28;
        }
        return retAngle;
    }

    bool Wpv3LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        // ROS_WARN("[WPV3]computeVelocityCommands() ");
        if(!m_bInitialized)
        {
            ROS_ERROR("wpv3_local_planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        int path_num = m_global_plan.size();
        if(path_num == 0)
        {
            return false;
        }

        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;

        /////////////////////////////////////////////////
        if(m_bFirstStep == true)
        {
            double target_x, target_y, target_th;
            while(m_nPathIndex < path_num-1)
            {
                getTransformedPosition(m_global_plan[m_nPathIndex], m_robot_base_frame_id, target_x, target_y, target_th);
                if(sqrt(target_x*target_x + target_y*target_y) < m_goal_dist_tolerance)
                {
                    m_nPathIndex ++;
                    //ROS_WARN("[WPV3-GOTO]target = %d ",m_nPathIndex);
                }
                else
                {
                    break;  //target is far enough
                }
            }

            double face_target = CalDirectAngle(0, 0, target_x, target_y);
            face_target = AngleFix(face_target,-2.1,2.1);
            if(fabs(face_target)> 0.09)
            {
                //turn in place
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = face_target * m_acc_scale_rot;
                if(cmd_vel.angular.z > 0) cmd_vel.angular.z +=0.2;
                if(cmd_vel.angular.z < 0) cmd_vel.angular.z -=0.2;
            }
            else
            {
                m_bFirstStep = false;
            }
        }
        ////////////////////////////////////////////////////////
        
        if(m_nStep == WPV3_STEP_ARRIVED)
        {
            ROS_WARN("[WPV3_ARRIVED](%.2f %.2f):%.2f",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z);
            return true;
        }

         /////////////////////////////////////////////////////////////
        //getTransformedPosition(m_global_plan[m_nPathIndex], m_robot_base_frame_id, gx, gy, gth);
        double goal_x,goal_y,goal_th;
        getTransformedPosition(m_global_plan.back(), m_robot_base_frame_id, goal_x, goal_y, goal_th);
        //ROS_WARN("goal(%.2f dy= %.2f) th= %.2f",goal_x, goal_y, goal_th);

        //  double face_goal = CalDirectAngle(0, 0, goal_x, goal_y);
        //  face_goal = AngleFix(face_goal,-2.1,2.1);
        //  ROS_WARN("face = %.2f goal(%.2f dy= %.2f) th= %.2f",face_goal, goal_x, goal_y, goal_th);
        
        if(m_nStep == WPV3_STEP_GOTO)
        {
            // check if global goal is near
            double goal_dist = sqrt(goal_x*goal_x + goal_y*goal_y);
            if(goal_dist < m_goal_dist_tolerance)
            {
                m_nStep = WPV3_STEP_NEAR;
                //ROS_WARN("[WPV3-GOTO] -> [WPV3_NEAR] (%.2f,%.2f) %.2f",goal_x, goal_y, goal_th);
            }
            else
            {
                //check if target is near
                double target_x, target_y, target_th;
                while(m_nPathIndex < path_num-1)
                {
                    getTransformedPosition(m_global_plan[m_nPathIndex], m_robot_base_frame_id, target_x, target_y, target_th);
                    if(sqrt(target_x*target_x + target_y*target_y) < m_goal_dist_tolerance)
                    {
                        m_nPathIndex ++;
                        //ROS_WARN("[WPV3-GOTO]target = %d ",m_nPathIndex);
                    }
                    else
                    {
                        break;  //target is far enough
                    }
                }

                double face_target = CalDirectAngle(0, 0, target_x, target_y);
                face_target = AngleFix(face_target,-2.1,2.1);
                if(fabs(face_target)> 0.52)
                {
                    //turn in place
                    cmd_vel.linear.x = 0;
                    cmd_vel.linear.y = 0;
                    cmd_vel.angular.z = face_target * m_acc_scale_rot;
                    if(cmd_vel.angular.z > 0) cmd_vel.angular.z +=0.2;
                    if(cmd_vel.angular.z < 0) cmd_vel.angular.z -=0.2;
                }
                else
                {
                    double target_dist = sqrt(target_x*target_x + target_y*target_y);
                    cmd_vel.linear.x = target_dist*cos(face_target) * m_acc_scale_trans;
                    cmd_vel.linear.y = target_dist*sin(face_target) * m_acc_scale_trans;
                    cmd_vel.angular.z = face_target * m_acc_scale_rot;
                    if(cmd_vel.linear.x > 0) cmd_vel.linear.x+=0.05;
                    if(cmd_vel.linear.x < 0) cmd_vel.linear.x-=0.05;
                    if(cmd_vel.linear.y > 0) cmd_vel.linear.y+=0.02;
                    if(cmd_vel.linear.y < 0) cmd_vel.linear.y-=0.02;

                    // cmd_vel.linear.x = 0;
                    // cmd_vel.linear.y = 0;
                    if(m_bAC == true)
                    {
                        m_bAC = false;
                        cmd_vel.linear.x = 0;
                        cmd_vel.linear.y = 0;
                        cmd_vel.angular.z = 0;
                        return m_bAC;
                    }
                }
                m_pub_target.publish(m_global_plan[m_nPathIndex]);
                // cmd_vel.linear.x += ac_x;
                // cmd_vel.linear.y += ac_y;
            }
        }

        if(m_nStep == WPV3_STEP_NEAR)
        {
            
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = goal_th;

            if(fabs(goal_th) < m_goal_yaw_tolerance)
            {
                m_goal_reached = true;
                m_nStep = WPV3_STEP_ARRIVED;
                cmd_vel.angular.z = 0;
                ROS_WARN("[WPV3-ARRIVED] goal (%.2f,%.2f) %.2f",goal_x, goal_y, goal_th);
            }
            m_pub_target.publish(m_global_plan.back());
        }

        if(cmd_vel.linear.x > m_max_vel_trans) cmd_vel.linear.x = m_max_vel_trans;
        if(cmd_vel.linear.x < -m_max_vel_trans) cmd_vel.linear.x = -m_max_vel_trans;
        if(cmd_vel.linear.y > m_max_vel_trans) cmd_vel.linear.y = m_max_vel_trans;
        if(cmd_vel.linear.y < -m_max_vel_trans) cmd_vel.linear.y = -m_max_vel_trans;
        if(cmd_vel.angular.z > m_max_vel_rot) cmd_vel.angular.z = m_max_vel_rot;
        if(cmd_vel.angular.z < -m_max_vel_rot) cmd_vel.angular.z = -m_max_vel_rot;

        SmothVel(cmd_vel);
        m_last_cmd = cmd_vel;
        
        return true;
    }

    void Wpv3LocalPlanner::SmothVel(geometry_msgs::Twist& cmd_vel)
    {
        float l_acc = 0.02;
        float dx = cmd_vel.linear.x - m_last_cmd.linear.x;
        if(dx > l_acc) dx = l_acc;
        if(dx < -1*l_acc) dx = -1*l_acc;
        float dy = cmd_vel.linear.y - m_last_cmd.linear.y;
        if(dy > l_acc) dy = l_acc;
        if(dy < -1*l_acc) dy = -1*l_acc;

        float a_acc = 0.1;
        float dz = cmd_vel.angular.z - m_last_cmd.angular.z;
        if(dz > a_acc) dz = a_acc;
        if(dz < -1*a_acc) dz = -1*a_acc;

        cmd_vel.linear.x = m_last_cmd.linear.x + dx;
        cmd_vel.linear.y = m_last_cmd.linear.y + dy;
        cmd_vel.angular.z = m_last_cmd.angular.z + dz;
    }


    bool Wpv3LocalPlanner::isGoalReached()
    {
        //ROS_WARN("[WPR1]isGoalReached() ");
        if (m_goal_reached)
        {
            ROS_INFO("GOAL Reached!");
            return true;
        }
        return false;
    }

    void Wpv3LocalPlanner::getTransformedPosition(geometry_msgs::PoseStamped& pose, std::string& frame_id, double& x, double& y, double& theta)
    {
        geometry_msgs::PoseStamped tf_pose;
        pose.header.stamp = ros::Time(0);
        m_tf_listener->transformPose(frame_id, pose, tf_pose);
        x = tf_pose.pose.position.x;
        y = tf_pose.pose.position.y,
        theta = tf::getYaw(tf_pose.pose.orientation);
    }

}