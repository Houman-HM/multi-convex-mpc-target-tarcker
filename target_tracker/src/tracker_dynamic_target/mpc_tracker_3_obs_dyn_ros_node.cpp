#include "target_tracker/tracker/fov_tracker.h"
#include <chrono>
#include <iostream>
#include <ctime>
#include <ratio>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "yaml-cpp/yaml.h"
#include <thread> 

using namespace Eigen;
using namespace std;
using namespace std::chrono;
using namespace optim;
ArrayXXf drone_pose = ArrayXXf(1,6);
ArrayXXf obs_1_pose = ArrayXXf(1,4);
ArrayXXf obs_2_pose = ArrayXXf (1,4);
ArrayXXf obs_3_pose = ArrayXXf(1,4); 
ArrayXXf target_pose = ArrayXXf(1,4); 

ArrayXXf drone_vel = ArrayXXf (1,4);
ArrayXXf target_vel = ArrayXXf (1,1);
ArrayXXf obs_1_vel = ArrayXXf (1,1);
ArrayXXf obs_2_vel = ArrayXXf (1,1);
ArrayXXf obs_3_vel = ArrayXXf (1,1);


bool is_received = false;
ros::Publisher drone_cmd_pub, obs_1_pub, obs_2_pub, obs_3_pub, target_pub;
float obs_1_vx, obs_1_vy, obs_2_vx, obs_2_vy, obs_3_vx, obs_3_vy, target_vx, target_vy, target_angular_v, obs_velocity_publisher_delay;


void callBack(const nav_msgs::Odometry::ConstPtr& drone_odom, const nav_msgs::Odometry::ConstPtr& obs_1_odom,
              const nav_msgs::Odometry::ConstPtr& obs_2_odom, const nav_msgs::Odometry::ConstPtr& obs_3_odom,
              const nav_msgs::Odometry::ConstPtr& target_odom)
{
  is_received = true;

  tf::Quaternion q_drone(
        drone_odom->pose.pose.orientation.x,
        drone_odom->pose.pose.orientation.y,
        drone_odom->pose.pose.orientation.z,
        drone_odom->pose.pose.orientation.w);
  tf::Matrix3x3 drone_orientation(q_drone);
  double drone_roll, drone_pitch, drone_yaw;
  drone_orientation.getRPY(drone_roll, drone_pitch, drone_yaw);

  tf::Quaternion q_target(
        target_odom->pose.pose.orientation.x,
        target_odom->pose.pose.orientation.y,
        target_odom->pose.pose.orientation.z,
        target_odom->pose.pose.orientation.w);
  tf::Matrix3x3 target_orientation(q_target);

  double target_roll, target_pitch, target_yaw;
  target_orientation.getRPY(target_roll, target_pitch, target_yaw);



  tf::Quaternion q_obs_1(
        obs_1_odom->pose.pose.orientation.x,
        obs_1_odom->pose.pose.orientation.y,
        obs_1_odom->pose.pose.orientation.z,
        obs_1_odom->pose.pose.orientation.w);
  tf::Matrix3x3 obs_1_orientation(q_obs_1);

  double obs_1_roll, obs_1_pitch, obs_1_yaw;
  obs_1_orientation.getRPY(obs_1_roll, obs_1_pitch, obs_1_yaw);


  tf::Quaternion q_obs_2(
        obs_2_odom->pose.pose.orientation.x,
        obs_2_odom->pose.pose.orientation.y,
        obs_2_odom->pose.pose.orientation.z,
        obs_2_odom->pose.pose.orientation.w);
  tf::Matrix3x3 obs_2_orientation(q_obs_2);

  double obs_2_roll, obs_2_pitch, obs_2_yaw;
  obs_2_orientation.getRPY(obs_2_roll, obs_2_pitch, obs_2_yaw);


  tf::Quaternion q_obs_3(
        obs_3_odom->pose.pose.orientation.x,
        obs_3_odom->pose.pose.orientation.y,
        obs_3_odom->pose.pose.orientation.z,
        obs_3_odom->pose.pose.orientation.w);
  tf::Matrix3x3 obs_3_orientation(q_obs_3);

  double obs_3_roll, obs_3_pitch, obs_3_yaw;
  obs_3_orientation.getRPY(obs_3_roll, obs_3_pitch, obs_3_yaw);

  drone_pose << drone_odom->pose.pose.position.x, drone_odom->pose.pose.position.y,
                drone_odom->pose.pose.position.z, drone_roll, drone_pitch, drone_yaw;

  drone_vel << drone_odom->twist.twist.linear.x, drone_odom->twist.twist.linear.y,
                drone_odom->twist.twist.linear.z, drone_odom->twist.twist.angular.z;

  obs_1_pose << obs_1_odom->pose.pose.position.x, obs_1_odom->pose.pose.position.y,obs_1_odom->pose.pose.position.z, obs_1_yaw;
  obs_2_pose << obs_2_odom->pose.pose.position.x, obs_2_odom->pose.pose.position.y, obs_2_odom->pose.pose.position.z, obs_2_yaw;
  obs_3_pose << obs_3_odom->pose.pose.position.x, obs_3_odom->pose.pose.position.y, obs_3_odom->pose.pose.position.z , obs_3_yaw;
  target_pose << target_odom->pose.pose.position.x, target_odom->pose.pose.position.y, target_odom->pose.pose.position.z, target_yaw;
  
  obs_1_vel << obs_1_odom->twist.twist.linear.x;
  obs_2_vel << obs_2_odom->twist.twist.linear.x;
  obs_3_vel << obs_3_odom->twist.twist.linear.x;
  target_vel << target_odom->twist.twist.linear.x;
}



void MPCThread()
{
 ProbData prob_data;


  prob_data.num_obs = 3;
  prob_data.num_samples = 100;
  prob_data.num = 100;
  prob_data.nvar = 11;
  prob_data.weight_smoothness = 80.0;

  prob_data.num_occ = prob_data.num_obs * prob_data.num_samples;
  prob_data.alpha_occ = ArrayXXf::Zero(prob_data.num_occ, prob_data.num);
  prob_data.d_occ = ArrayXXf::Ones(prob_data.num_occ, prob_data.num);

  prob_data.vx_min = -1.0;
  prob_data.vx_max = 1.0;
  prob_data.ax_min = -2.0;
  prob_data.ax_max = 2.0;

  prob_data.y_max =  3.0;
  prob_data.vy_min = -1.0;
  prob_data.vy_max = 1.0;
  prob_data.ay_min = -2.0;
  prob_data.ay_max = 2.0;

  prob_data.lamda_dyn_x = 0.0 * ArrayXXf::Ones(prob_data.nvar,1);
  prob_data.lamda_dyn_y = 0.0 * ArrayXXf::Ones(prob_data.nvar,1);

  ProbData sim_data;

  sim_data.x_init = -5.0;
  sim_data.vx_init = 0.0;
  sim_data.ax_init = 0.0;

  sim_data.y_init = 3.0;
  sim_data.vy_init = 0.0;
  sim_data.ay_init = 0.0;


  sim_data.vx_fin = 0.0;
  sim_data.vy_fin = 0.0;

  sim_data.ax_fin = 0.0;
  sim_data.ay_fin = 0.0;

  sim_data.x_obs_1_init = -3.5;
  sim_data.y_obs_1_init = 4.18;


  sim_data.x_obs_2_init = -6.0;
  sim_data.y_obs_2_init = 5.6;

  sim_data.x_obs_3_init = -3.36;
  sim_data.y_obs_3_init = 5.2;

  sim_data.obs_1_vx = 0.3;
  sim_data.obs_1_vy = 0.0;

  sim_data.obs_2_vx = 0.3;
  sim_data.obs_2_vy = 0.0;

  sim_data.obs_3_vx = 0.3;
  sim_data.obs_3_vy = 0.0;


  sim_data.x_target_init = -4.0;
  sim_data.y_target_init =  2.0;
  sim_data.theta_target_init = 0.0;

  sim_data.v_target = 0.5;
  sim_data.omega_target = 0.2;

  sim_data.alpha_init = atan2(sim_data.y_target_init - sim_data.y_init, sim_data.x_target_init-sim_data.x_init);
  sim_data.alphadot_init = 0.0;


  prob_data.alpha_fov = ArrayXXf::Zero(prob_data.num,1);
  prob_data.d_fov = ArrayXXf::Ones(prob_data.num,1);
  prob_data.d_fov_min = 2.0;
  prob_data.d_fov_max = 2.5;


  float t_update = 0.005;
  float t_up = 0.01;
  int mpc_iter = 70000;
  float t_fin = 5.0;
  int num_up = int(t_fin/t_up);

  ArrayXXf tot_time_up = linspace(0.0, t_fin, num_up);
  ArrayXXf tot_time_copy_up = reshape(tot_time_up,num_up, 1);
  ThreeVar ppp_up = optim::bernstein_coeff_order10_new(10, tot_time_copy_up(0), tot_time_copy_up(tot_time_copy_up.rows()-1), tot_time_copy_up);

  prob_data.t = t_fin/prob_data.num;
  ArrayXXf tot_time = linspace(0.0, t_fin, prob_data.num);
  ArrayXXf tot_time_copy = reshape(tot_time, prob_data.num,1);
  ThreeVar ppp  = optim::bernstein_coeff_order10_new(10, tot_time_copy(0), tot_time_copy(tot_time_copy.rows()-1), tot_time_copy);
  ArrayXXf u = linspace(0.0001, 0.99, prob_data.num_samples);
  prob_data.A_occ = compute_A_occ(ppp.first, prob_data, u);
  prob_data.A_fov = ppp.first;



  ArrayXXf x_drone = sim_data.x_init * ArrayXXf::Ones(1, mpc_iter);
  ArrayXXf y_drone = sim_data.y_init * ArrayXXf::Ones(1, mpc_iter);

  ArrayXXf  alpha_drone = sim_data.alpha_init * ArrayXXf::Ones(1, mpc_iter);
  ArrayXXf  alphadot_drone = sim_data.alphadot_init * ArrayXXf::Ones(1, mpc_iter);

  ArrayXXf ax_drone = sim_data.ax_init * ArrayXXf::Ones(1, mpc_iter);
  ArrayXXf ay_drone = sim_data.ay_init * ArrayXXf::Ones(1, mpc_iter);

  ArrayXXf vx_drone = sim_data.vx_init * ArrayXXf::Ones(1, mpc_iter);
  ArrayXXf vy_drone = sim_data.vy_init * ArrayXXf::Ones(1, mpc_iter);

  ArrayXXf vx_local_drone = sim_data.vx_init * ArrayXXf::Ones(1, mpc_iter);
  ArrayXXf vy_local_drone = sim_data.vy_init * ArrayXXf::Ones(1, mpc_iter);

  ArrayXXf x_target_pos = sim_data.x_target_init * ArrayXXf::Ones(1, mpc_iter);
  ArrayXXf y_target_pos = sim_data.y_target_init * ArrayXXf::Ones(1, mpc_iter);
  ArrayXXf theta_target_pos = sim_data.theta_target_init * ArrayXXf::Ones(1, mpc_iter);

  ArrayXXf x_obs_1_pos = sim_data.x_obs_1_init* ArrayXXf::Ones(1, mpc_iter);
  ArrayXXf y_obs_1_pos = sim_data.y_obs_1_init* ArrayXXf::Ones(1, mpc_iter);

  ArrayXXf x_obs_2_pos = sim_data.x_obs_2_init * ArrayXXf::Ones(1, mpc_iter);
  ArrayXXf y_obs_2_pos = sim_data.y_obs_2_init * ArrayXXf::Ones(1, mpc_iter);

  ArrayXXf x_obs_3_pos = sim_data.x_obs_3_init * ArrayXXf::Ones(1, mpc_iter);
  ArrayXXf y_obs_3_pos = sim_data.y_obs_3_init * ArrayXXf::Ones(1, mpc_iter);

  obs_1_vx = sim_data.obs_1_vx;
  obs_1_vy = sim_data.obs_1_vy;
  obs_2_vx = sim_data.obs_2_vx;
  obs_2_vy = sim_data.obs_2_vy;
  obs_3_vx = sim_data.obs_3_vx;
  obs_3_vy = sim_data.obs_3_vy;
  target_vx = sim_data.v_target;
  target_angular_v = sim_data.omega_target;

  prob_data.rho_occ = 1.0;
  prob_data.rho_fov = 2.0;
  int maxiter;
  float delt_factor = 1.0;

  while (!is_received && ros::ok())
    {
      ros::Duration(0.001).sleep();
    }

  geometry_msgs::Twist drone_cmd;
  for (size_t i = 1; i <mpc_iter; i++)
  {
    if(i==1)
    {
      prob_data.maxiter = 1;
      maxiter = 1;
    }
    if(i>1)
    {
      prob_data.maxiter = 1;
      maxiter = 1;
    }
    

    sim_data.obs_1_vx = obs_1_vel(0);

    sim_data.obs_2_vx = obs_2_vel(0);

    sim_data.obs_3_vx = obs_3_vel(0);

    sim_data.vx_target = target_vel(0) * cos(target_pose(3));
    sim_data.vy_target = target_vel(0) * sin(target_pose(3));

    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    DroneVelocities drone_velocities = optim::main_fov(prob_data, sim_data, i, maxiter, delt_factor, ppp_up.first,
    ppp_up.second, ppp_up.third, t_update, t_up, ppp.first, ppp.second, ppp.third, tot_time, u, tot_time_up);
    
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << time_span.count() << std::endl;

    prob_data  = drone_velocities.prob_data;
    
    vx_drone(i) = drone_velocities.vx_drone;
    vy_drone(i) = drone_velocities.vy_drone;
    ax_drone(i) = drone_velocities.ax_drone;
    ay_drone(i) = drone_velocities.ay_drone;
    alphadot_drone(i) = drone_velocities.alphadot_drone;
    vx_local_drone(i) = drone_velocities.vx_local_drone;
    vy_local_drone(i) = drone_velocities.vy_local_drone;


    drone_cmd.linear.x = vx_local_drone(i);
    drone_cmd.linear.y = vy_local_drone(i);
    drone_cmd.angular.z = alphadot_drone(i);
    drone_cmd_pub.publish(drone_cmd);

    x_drone(i) = drone_pose(0);
    y_drone(i) = drone_pose(1);
    alpha_drone(i) = drone_pose(5); 

    x_target_pos(i) = target_pose(0); 
    y_target_pos(i) = target_pose(1);

    sim_data.x_init = x_drone(i);
    sim_data.vx_init = vx_drone(i);
    sim_data.ax_init = ax_drone(i);

    sim_data.y_init = y_drone(i);
    sim_data.vy_init = vy_drone(i);
    sim_data.ay_init = ay_drone(i);

    sim_data.alpha_init = alpha_drone(i);


    sim_data.y_target_init = y_target_pos(i);
    sim_data.x_target_init = x_target_pos(i);

    sim_data.x_obs_1_init = obs_1_pose(0);
    sim_data.y_obs_1_init = obs_1_pose(1);

    sim_data.x_obs_2_init = obs_2_pose(0);
    sim_data.y_obs_2_init = obs_2_pose(1);

    sim_data.x_obs_3_init = obs_3_pose(0);
    sim_data.y_obs_3_init = obs_3_pose(1);
  }
}

void obstacleVelPublisherThread()
{
  geometry_msgs::Twist obs_1_vel;
  geometry_msgs::Twist obs_2_vel;
  geometry_msgs::Twist obs_3_vel;
  geometry_msgs::Twist target_vel;
  double omega = 1.0;
  while(ros::ok())
  {
    double vx = 0.3 * sin(omega);
    obs_1_vel.linear.x = -vx;
    obs_2_vel.linear.x = vx; 
    obs_3_vel.linear.x = vx;

    obs_1_pub.publish(obs_1_vel);
    obs_2_pub.publish(obs_2_vel);
    obs_3_pub.publish(obs_3_vel);
    obs_1_pub.publish(obs_1_vel);
    obs_2_pub.publish(obs_2_vel);
    obs_3_pub.publish(obs_3_vel);
    ros::Duration(10).sleep();
    omega = omega * -1;
  }
}
int main(int argc, char **argv)
{
ros::init(argc, argv, "mpc_node_cpp");
ros::NodeHandle n;

drone_cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
obs_1_pub = n.advertise<geometry_msgs::Twist>("obs_1/cmd_vel", 1000);
obs_2_pub = n.advertise<geometry_msgs::Twist>("obs_2/cmd_vel", 1000);
obs_3_pub = n.advertise<geometry_msgs::Twist>("obs_3/cmd_vel", 1000);
target_pub = n.advertise<geometry_msgs::Twist>("target/cmd_vel", 1000);
message_filters::Subscriber<nav_msgs::Odometry> drone_odom_sub(n, "/bebop/odom", 1000);
message_filters::Subscriber<nav_msgs::Odometry> obs_1_odom_sub(n, "/obs_1/odom", 1000);
message_filters::Subscriber<nav_msgs::Odometry> obs_2_odom_sub(n, "/obs_2/odom", 1000);
message_filters::Subscriber<nav_msgs::Odometry> obs_3_odom_sub(n, "/obs_3/odom", 1000);
message_filters::Subscriber<nav_msgs::Odometry> target_odom_sub(n, "/target/odom", 1000);
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry,
                                                         nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), drone_odom_sub, obs_1_odom_sub, obs_2_odom_sub, obs_3_odom_sub, target_odom_sub);
sync.registerCallback(boost::bind(&callBack, _1, _2, _3,_4, _5));
std::thread mpc_thread = std::thread(MPCThread);
std::thread obs_velocity_thread = std::thread(obstacleVelPublisherThread);
ros::spin();

return 0;
}

