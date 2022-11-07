#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <ctgmath>
#include <math.h>

#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

int main(int argc, char** argv)
{

  //Costanti
  double pi = 3.14159265359;
  double S = 0.05;
  double deltaT = 0.0025;
  double EPSILON = 0.0000001;
  int N =  20;
  int C =  5;
  
  //Variabili
  double T = 0.025; //T=0.05
  double rho=T, theta=0;
  double x=0,y=0,z=0;
  bool z_changed=true;
  int plane_index = 0;

  ros::init (argc, argv, "uvc_light_model");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("uvc_pointcloud", 1);

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "camera_depth_optical_frame";
  msg->height = 20;
  msg->width = 100;
  msg->is_dense = false; 
  msg->points.resize(msg->height * msg->width);

  for (auto& point: msg->points)    //for (auto& point: msg->points) 
  {
    if(z_changed)
    {
      z += S;
      z_changed=false;
      plane_index++;
      //std::cerr << "Z cambiata, e siamo al piano numero: " << plane_index <<"\n";
    }
    x = rho*cos(theta); //cos
    y = rho*sin(theta); //sen

    point.z = z;
    point.x = x;
    point.y = y;
    point.r = 255; //plane_index*10
    point.g = 64; //plane_index*10
    point.b = plane_index*10; 

    theta+=2*pi/N;

    if(abs(theta - 2*pi)<EPSILON){ //Cioè theta è circa 2*pi
        theta = 0;
        if(rho==C*T){
            //T = 0.7*T; --> In questa maniera la riduzione non è costante da piano a piano, si ottiene non un cono ma una discesa geometrica
            T += deltaT; //T -= deltaT;
            rho = T;
            z_changed = true;
        } else {
            rho+=T;     //rho+=T;
            //z_changed=false;
        }
    }
  }

  ros::Rate loop_rate(1);
  while (nh.ok())
  {
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}