#include "target_tracker/2_obs/fov_const_drone_eigen.h"
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
#include <ros/package.h>

using namespace Eigen;
using namespace std;
using namespace std::chrono;
using namespace optim;
ArrayXXf drone_pose = ArrayXXf(1,6);
ArrayXXf obs_1_pose = ArrayXXf(1,3);
ArrayXXf obs_2_pose = ArrayXXf (1,3);
ArrayXXf obs_3_pose = ArrayXXf(1,3); 
ArrayXXf drone_vel = ArrayXXf (1,4);
bool is_received = false;
ros::Publisher drone_cmd_pub, obs_1_pub, obs_2_pub, real_bebop, jackal_pub, robotont_1_pub, robotont_2_pub;
float obs_1_vx, obs_1_vy, obs_2_vx, obs_2_vy, obs_velocity_publisher_delay;
float bot_1_vx, bot_1_vy, bot_2_vx, bot_2_vy;


void callBack(const nav_msgs::Odometry::ConstPtr& drone_odom, const nav_msgs::Odometry::ConstPtr& obs_1_odom,
              const nav_msgs::Odometry::ConstPtr& obs_2_odom)
{
  is_received = true;
  tf::Quaternion q(
        drone_odom->pose.pose.orientation.x,
        drone_odom->pose.pose.orientation.y,
        drone_odom->pose.pose.orientation.z,
        drone_odom->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  drone_pose << drone_odom->pose.pose.position.x, drone_odom->pose.pose.position.y,
                drone_odom->pose.pose.position.z, roll, pitch, yaw;

  drone_vel << drone_odom->twist.twist.linear.x, drone_odom->twist.twist.linear.y,
                drone_odom->twist.twist.linear.z, drone_odom->twist.twist.angular.z;

  obs_1_pose << obs_1_odom->pose.pose.position.x, obs_1_odom->pose.pose.position.y,
                obs_1_odom->pose.pose.position.z;

  obs_2_pose << obs_2_odom->pose.pose.position.x, obs_2_odom->pose.pose.position.y, obs_2_odom->pose.pose.position.z;
}



void MPCThread()
{
  std::string package_path = ros::package::getPath("target_tracker");
  YAML::Node config = YAML::LoadFile(package_path + "/params/2_obs_params.yaml");
  ProbData prob_data;
  ProbData sim_data;

  prob_data.r_1 = ArrayXXf(3, 1);
  prob_data.r_1 << 5, 0.0, 1.0;

  prob_data.num_obs = config["Params"]["prob_data.num_obs"].as<int>();
  prob_data.num_samples = config["Params"]["prob_data.num_samples"].as<int>();
  prob_data.num = config["Params"]["prob_data.num"].as<int>();
  prob_data.nvar = config["Params"]["prob_data.nvar"].as<int>();
  prob_data.weight_smoothness = config["Params"]["prob_data.weight_smoothness"].as<float>();

  prob_data.x_min = config["Params"]["prob_data.x_min"].as<float>();
  prob_data.x_max =  config["Params"]["prob_data.x_max"].as<float>();
  prob_data.vx_min = config["Params"]["prob_data.vx_min"].as<float>();
  prob_data.vx_max = config["Params"]["prob_data.vx_max"].as<float>();
  prob_data.ax_min = config["Params"]["prob_data.ax_min"].as<float>();
  prob_data.ax_max = config["Params"]["prob_data.ax_max"].as<float>();


  prob_data.y_min = config["Params"]["prob_data.y_min"].as<float>();
  prob_data.y_max = config["Params"]["prob_data.y_max"].as<float>();
  prob_data.vy_min = config["Params"]["prob_data.vy_min"].as<float>();
  prob_data.vy_max = config["Params"]["prob_data.vy_max"].as<float>();
  prob_data.ay_min = config["Params"]["prob_data.ay_min"].as<float>();
  prob_data.ay_max = config["Params"]["prob_data.ay_max"].as<float>();


  sim_data.x_init = config["Params"]["sim_data.x_init"].as<float>();
  sim_data.vx_init = config["Params"]["sim_data.vx_init"].as<float>();
  sim_data.ax_init = config["Params"]["sim_data.ax_init"].as<float>();

  sim_data.y_init = config["Params"]["sim_data.y_init"].as<float>();
  sim_data.vy_init = config["Params"]["sim_data.vy_init"].as<float>();
  sim_data.ay_init =config["Params"]["sim_data.ay_init"].as<float>();


  sim_data.z_init = config["Params"]["sim_data.z_init"].as<float>();
  sim_data.vz_init = config["Params"]["sim_data.vz_init"].as<float>();
  sim_data.az_init = config["Params"]["sim_data.az_init"].as<float>();

  sim_data.vx_fin = config["Params"]["sim_data.vx_fin"].as<float>();
  sim_data.vy_fin = config["Params"]["sim_data.vy_fin"].as<float>();
  sim_data.vz_fin = config["Params"]["sim_data.vz_fin"].as<float>();

  sim_data.ax_fin = config["Params"]["sim_data.ax_fin"].as<float>();
  sim_data.ay_fin = config["Params"]["sim_data.ay_fin"].as<float>();
  sim_data.az_fin =config["Params"]["sim_data.az_fin"].as<float>();

  sim_data.x_fin = config["Params"]["sim_data.x_fin"].as<float>();
  sim_data.y_fin = config["Params"]["sim_data.y_fin"].as<float>();


  sim_data.alpha_init = atan2(prob_data.r_1(1) - sim_data.y_init, prob_data.r_1(0) - sim_data.x_init);
  sim_data.alphadot_init = config["Params"]["sim_data.alphadot_init"].as<float>();

  sim_data.x_obs_1_init = config["Params"]["sim_data.x_obs_1_init"].as<float>();
  sim_data.y_obs_1_init = config["Params"]["sim_data.y_obs_1_init"].as<float>();

  sim_data.x_obs_2_init = config["Params"]["sim_data.x_obs_2_init"].as<float>();
  sim_data.y_obs_2_init = config["Params"]["sim_data.y_obs_2_init"].as<float>();

  sim_data.obs_1_vx = config["Params"]["sim_data.obs_1_vx"].as<float>();
  sim_data.obs_1_vy = config["Params"]["sim_data.obs_1_vy"].as<float>();

  sim_data.obs_2_vx = config["Params"]["sim_data.obs_2_vx"].as<float>();
  sim_data.obs_2_vy = config["Params"]["sim_data.obs_2_vy"].as<float>();


  prob_data.t_update = config["Params"]["prob_data.t_update"].as<float>();
  prob_data.t_up = config["Params"]["prob_data.t_up"].as<float>();
  prob_data.mpc_iter = config["Params"]["prob_data.mpc_iter"].as<int>();
  prob_data.t_fin  = config["Params"]["prob_data.t_fin"].as<float>();
  int maxiter  = config["Params"]["maxiter"].as<int>();
  obs_velocity_publisher_delay = config["Params"]["obs_velocity_publisher_delay"].as<float>();
  float mpc_delay = config["Params"]["mpc_delay"].as<float>();


  prob_data.rho_obs = config["Params"]["prob_data.rho_obs"].as<float>();
  prob_data.rho_occ = config["Params"]["prob_data.rho_occ"].as<float>();
  prob_data.rho_ineq = config["Params"]["prob_data.rho_ineq"].as<float>();
  float delt_factor = config["Params"]["delta_factor"].as<float>();

  prob_data.num_occ = (prob_data.num_obs-1) *  prob_data.num_samples;

  prob_data.alpha_obs = ArrayXXf::Zero(prob_data.num_obs, prob_data.num);
  prob_data.d_obs = ArrayXXf::Ones(prob_data.num_obs, prob_data.num);

  prob_data.alpha_occ = ArrayXXf::Zero (prob_data.num_occ, prob_data.num);
  prob_data.d_occ = ArrayXXf::Ones(prob_data.num_occ, prob_data.num);
  prob_data.s_ineq_x = ArrayXXf::Ones(1, 6 * prob_data.num);

  prob_data.s_ineq_x = hstack(hstack(hstack(hstack(hstack(prob_data.x_max * ArrayXXf::Ones(1, prob_data.num), -prob_data.x_min * ArrayXXf::Ones(1, prob_data.num)), 
  prob_data.vx_max * ArrayXXf::Ones(1, prob_data.num)), -prob_data.vx_min * ArrayXXf::Ones(1, prob_data.num)), prob_data.ax_max * ArrayXXf::Ones(1, prob_data.num)),
                      -prob_data.ax_min * ArrayXXf::Ones(1, prob_data.num));

  prob_data.lamda_ineq_x = 0.0 * ArrayXXf::Ones(1, 6*prob_data.num);

  prob_data.s_ineq_y = hstack(hstack(hstack(hstack(hstack(prob_data.y_max * ArrayXXf::Ones(1, prob_data.num), -prob_data.y_min * ArrayXXf::Ones(1, prob_data.num)),
  prob_data.vy_max * ArrayXXf::Ones(1, prob_data.num)), -prob_data.vy_min * ArrayXXf::Ones(1, prob_data.num)), prob_data.ay_max * ArrayXXf::Ones(1, prob_data.num)),
                        -prob_data.ay_min * ArrayXXf::Ones(1, prob_data.num));

  prob_data.lamda_ineq_y = 0.0 * ArrayXXf::Ones(1, 6*prob_data.num);

  prob_data.s_ineq_z = ArrayXXf::Ones(1, 6 * prob_data.num);
  prob_data.lamda_ineq_z = ArrayXXf::Ones (1, 6*prob_data.num);

  prob_data.lamda_dyn_x = 0.0 * ArrayXXf::Ones(prob_data.nvar,1);
  prob_data.lamda_dyn_y = 0.0 * ArrayXXf::Ones(prob_data.nvar,1);
  prob_data.lamda_dyn_z = 0.0 * ArrayXXf::Ones(prob_data.nvar,1);


  int num_up = int(prob_data.t_fin/prob_data.t_up);

  ArrayXXf tot_time_up = linspace(0.0, prob_data.t_fin , num_up);
  ArrayXXf tot_time_copy_up = reshape(tot_time_up,num_up, 1);
  ThreeVar ppp_up = optim::bernstein_coeff_order10_new(10, tot_time_copy_up(0), tot_time_copy_up(tot_time_copy_up.rows()-1), tot_time_copy_up);

  prob_data.t = prob_data.t_fin /prob_data.num;
  ArrayXXf tot_time = linspace(0.0, prob_data.t_fin , prob_data.num);
  ArrayXXf tot_time_copy = reshape(tot_time, prob_data.num,1);
  ThreeVar ppp  = optim::bernstein_coeff_order10_new(10, tot_time_copy(0), tot_time_copy(tot_time_copy.rows()-1), tot_time_copy);
  ArrayXXf u = linspace(0.0001, 0.99, prob_data.num_samples);
  prob_data.A_occ = compute_A_occ(ppp.first, prob_data, u);



  ArrayXXf x_drone = sim_data.x_init * ArrayXXf::Ones(1, prob_data.mpc_iter);
  ArrayXXf y_drone = sim_data.y_init * ArrayXXf::Ones(1, prob_data.mpc_iter);
  ArrayXXf z_drone = sim_data.z_init * ArrayXXf::Ones(1, prob_data.mpc_iter);

  ArrayXXf  alpha_drone = sim_data.alpha_init * ArrayXXf::Ones(1, prob_data.mpc_iter);
  ArrayXXf  alphadot_drone = sim_data.alphadot_init * ArrayXXf::Ones(1, prob_data.mpc_iter);

  ArrayXXf ax_drone = sim_data.ax_init * ArrayXXf::Ones(1, prob_data.mpc_iter);
  ArrayXXf ay_drone = sim_data.ay_init * ArrayXXf::Ones(1, prob_data.mpc_iter);
  ArrayXXf az_drone = sim_data.az_init * ArrayXXf::Ones(1, prob_data.mpc_iter);


  ArrayXXf vx_drone = sim_data.vx_init * ArrayXXf::Ones(1, prob_data.mpc_iter);
  ArrayXXf vy_drone = sim_data.vy_init * ArrayXXf::Ones(1, prob_data.mpc_iter);
  ArrayXXf vz_drone = sim_data.vz_init * ArrayXXf::Ones(1, prob_data.mpc_iter);

  ArrayXXf x_obs_1_pos = sim_data.x_obs_1_init* ArrayXXf::Ones(1, prob_data.mpc_iter);
  ArrayXXf y_obs_1_pos = sim_data.y_obs_1_init* ArrayXXf::Ones(1, prob_data.mpc_iter);

  ArrayXXf x_obs_2_pos = sim_data.x_obs_2_init * ArrayXXf::Ones(1, prob_data.mpc_iter);
  ArrayXXf y_obs_2_pos = sim_data.y_obs_2_init * ArrayXXf::Ones(1, prob_data.mpc_iter);

  ArrayXXf vx_local_drone = sim_data.vx_init * ArrayXXf::Ones(1, prob_data.mpc_iter);
  ArrayXXf vy_local_drone = sim_data.vy_init * ArrayXXf::Ones(1, prob_data.mpc_iter);
    
  
  
  obs_1_vx = sim_data.obs_1_vx;
  obs_1_vy = sim_data.obs_1_vy;
  obs_2_vx = sim_data.obs_2_vx;
  obs_2_vy = sim_data.obs_2_vy;
  geometry_msgs::Twist drone_cmd;
  while (!is_received && ros::ok())
  {
    ros::Duration(0.001).sleep();
  }
  ros::Duration(mpc_delay).sleep();

  for (size_t i = 1; i <prob_data.mpc_iter; i++)
  {
    if(i==1)
    {
      prob_data.maxiter = maxiter;
      delt_factor = 1.0;
    }
    if(i>1)
    {
      prob_data.maxiter = maxiter;
      delt_factor = 1.0;
    }

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    DroneVelocities drone_velocities = optim::main_fov(prob_data, sim_data, i, maxiter, delt_factor, ppp_up.first,
    ppp_up.second, ppp_up.third, prob_data.t_update, prob_data.t_up, ppp.first, ppp.second, ppp.third, tot_time, u);
    
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

    alpha_drone(i) = drone_pose(5); //############ replace by ROS feedback

    x_obs_1_pos(i) = obs_1_pose(0); //####### ROS feedback
    y_obs_1_pos(i) = obs_1_pose(1); //### ROS feedback

    x_obs_2_pos(i) = obs_2_pose(0); //####### ROS feedback
    y_obs_2_pos(i) = obs_2_pose(1);// ### ROS feedback


    sim_data.x_init = x_drone(i);
    sim_data.vx_init = vx_drone(i);
    sim_data.ax_init = ax_drone(i);

    sim_data.y_init = y_drone(i);
    sim_data.vy_init = vy_drone(i);
    sim_data.ay_init = ay_drone(i);

    sim_data.alpha_init = alpha_drone(i);

    sim_data.y_obs_1_init = y_obs_1_pos(i);
    sim_data.x_obs_1_init = x_obs_1_pos(i);

    sim_data.y_obs_2_init = y_obs_2_pos(i);
    sim_data.x_obs_2_init = x_obs_2_pos(i);
  }
}


void obstacleVelPublisherThread()
{
  geometry_msgs::Twist obs_1_vel;
  geometry_msgs::Twist obs_2_vel;
  ros::Duration(obs_velocity_publisher_delay).sleep();
  float ratio = 0.01;
  while(ros::ok())
  {
    obs_1_vel.linear.x = obs_1_vx;
    obs_1_vel.linear.y = obs_1_vy;
    obs_2_vel.linear.x = obs_2_vx;
    obs_2_vel.linear.y = obs_2_vy;

    obs_1_pub.publish(obs_1_vel);
    obs_2_pub.publish(obs_2_vel);
    ros::Duration(0.05).sleep();
  }
}
int main(int argc, char **argv)
{
ros::init(argc, argv, "mpc_node_cpp");
ros::NodeHandle n;

drone_cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
obs_1_pub = n.advertise<geometry_msgs::Twist>("obs_1/cmd_vel", 1000);
obs_2_pub = n.advertise<geometry_msgs::Twist>("obs_2/cmd_vel", 1000);


message_filters::Subscriber<nav_msgs::Odometry> drone_odom_sub(n, "/bebop/odom", 1);
message_filters::Subscriber<nav_msgs::Odometry> obs_1_odom_sub(n, "/obs_1/odom", 1);
message_filters::Subscriber<nav_msgs::Odometry> obs_2_odom_sub(n, "/obs_2/odom", 1);
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), drone_odom_sub, obs_1_odom_sub, obs_2_odom_sub);
sync.registerCallback(boost::bind(&callBack, _1, _2, _3));
std::thread mpc_thread = std::thread(MPCThread);
std::thread obs_velocity_thread = std::thread(obstacleVelPublisherThread);
ros::spin();

return 0;
}
