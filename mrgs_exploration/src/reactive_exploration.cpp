  #include <stdlib.h>
  #include <stdio.h>
  #include <vector>
  #include <ros/ros.h>
  #include <geometry_msgs/Twist.h>
  #include <geometry_msgs/PointStamped.h>
  #include <geometry_msgs/Point32.h>
  #include <geometry_msgs/PolygonStamped.h>
  #include <sensor_msgs/LaserScan.h>
  #include <std_msgs/Float32.h>
  
  int num_points=0;
  
  ros::Publisher cmd_vel_pub;
  const double PI  =3.141592653589793238462;
  double ang_rotate=5*PI/180;
  double curr_ang=0;
  int rotation_lock=0;
  int waitToPublish = 0;
  
  
   void laserReceived(const sensor_msgs::LaserScan::ConstPtr& ls)
  {
      
    num_points= ls->ranges.size(); // VERIFICA
    float dist[num_points];
    float min_dist=10;
    float danger=0;
    size_t i;
    float sum1=0;
    float sum2=0;
    float unknown_rate=0;
    float ur_right=0;
    float ur_left=0;
    float sum_dist=0;
    
    for (i = 0; i < num_points; ++i)  
    {
      
      if(ls->ranges[i]>=ls->range_min && ls->ranges[i] <= ls->range_max)
	dist[i] = ls->ranges[i];
      else
	dist[i] = -1;
      
      
      sum_dist+=dist[i];
      
      if(dist[i]<min_dist && dist[i]>=ls->range_min)
	min_dist=dist[i];
      
      if(dist[i] <=ls->range_min+0.15 && dist[i] >=0)
	danger=1;
      
      if(i<num_points/2)
      {
	sum1+=dist[i];
	if(dist[i]==-1)
	  ur_left+=1;
	
      }
      else
      {
	sum2+=dist[i];
	if(dist[i]==-1)
	  ur_right+=1;
      }
      if(dist[i]==-1)
	unknown_rate++;
    }
    
    ur_left=ur_left/(num_points/2);
    ur_right=ur_right/(num_points/2);
    
    unknown_rate=unknown_rate/num_points;
  
    if(unknown_rate>=0.15)
      danger=1;
    
      geometry_msgs::Twist cmd_vel;
      
      if(danger==0)
      {
	rotation_lock=0;
	cmd_vel.linear.x=0.15;
	cmd_vel.angular.z=0.0;
	
      }
      else
      {
	
	cmd_vel.linear.x=0.0;
	if (rotation_lock==0)
	{
	    if(sum1<sum2) 	//retirei a utilização do unknown rate para a escohla da direção. nao deves preicsar com o laser
	      cmd_vel.angular.z=ang_rotate; 
	    else
	      cmd_vel.angular.z=-ang_rotate;
	  }
	  else
	    cmd_vel.angular.z=ang_rotate;
	  
	
	
	if(rotation_lock==1)
	  cmd_vel.angular.z=curr_ang;
	else
	  curr_ang=cmd_vel.angular.z;
	
	rotation_lock=1;
	
	}
	
      if(waitToPublish==0)
	cmd_vel_pub.publish(cmd_vel);
      
   
    
  }
  
  
  
  int main(int argc, char** argv)
  {
    
    
    ros::init(argc, argv, "reactive_explore");
    ros::NodeHandle n;
    ros::Rate loop_rate(10); // 30 Hz
        
    
    // ROS publishers and subscribers
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	ros::Subscriber laser_sub  = n.subscribe<sensor_msgs::LaserScan>("scan", 10, laserReceived);
	
	ros::spin(); //trigger callbacks and prevents exiting
	return(0);
  }
  
  
  
