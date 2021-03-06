#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>


using namespace std;

class TeleopJoy
{
public:
  TeleopJoy();
  bool bStart;
  float lx;
  float ly;
  float ry;
  ros::NodeHandle n;
  ros::Subscriber sub;

  ros::Time current_time;
  ros::Time last_time;
  ros::Publisher velcmd_pub;
  void SendVelcmd();
private:
  void callBack(const sensor_msgs::Joy::ConstPtr& joy);
};

TeleopJoy::TeleopJoy()
{
  lx = 0;
  ly = 0;
  ry = 0;
  bStart = false;
  velcmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
  sub = n.subscribe<sensor_msgs::Joy>("joy",10,&TeleopJoy::callBack,this);
  current_time = ros::Time::now();
  last_time = ros::Time::now();
 
  ROS_INFO("TeleopJoy");
}

static float kx = 0.3;
static float ky = 0.2;
static float kz = 0.3;
void TeleopJoy::callBack(const sensor_msgs::Joy::ConstPtr& joy)
{

  ROS_INFO("Joy: [%.2f , %.2f]", lx , ry);
  lx = joy->axes[1];
  ly = joy->axes[0];
  ry = joy->axes[3];

  bStart = true;
}

void TeleopJoy::SendVelcmd()
{
  if(bStart == false)
    return;
  geometry_msgs::Twist vel_cmd;
  vel_cmd.linear.x = (float)lx*kx;
  vel_cmd.linear.y = (float)ly*ky;
  vel_cmd.linear.z = 0;
  vel_cmd.angular.x = 0;
  vel_cmd.angular.y = 0;
  vel_cmd.angular.z = (float)ry*kz;
  velcmd_pub.publish(vel_cmd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wpv3_js_velcmd");

  TeleopJoy cTeleopJoy;

  ros::Rate r(30);
  while(ros::ok())
  {
    cTeleopJoy.SendVelcmd();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
