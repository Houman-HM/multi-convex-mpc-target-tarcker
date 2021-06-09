#ifndef TRACKER_EIGEN_2_OBS_H_
#define TRACKER_EIGEN_2_OBS_H_

#pragma once
#include <eigen3/Eigen/Dense>

  struct TwoVar
  {
    Eigen::ArrayXXf first;
    Eigen::ArrayXXf second;
  };

  struct ThreeVar
  {
    Eigen::ArrayXXf first;
    Eigen::ArrayXXf second;
    Eigen::ArrayXXf third;
  };

struct ProbData
  {
    float maxiter,x_init,vx_init,ax_init,y_init ,vy_init,ay_init,z_init,vz_init,az_init,
        x_fin,vx_fin,ax_fin,y_fin,vy_fin,ay_fin,z_fin,vz_fin,az_fi,alpha_init,alpha_fin,t_fin, weight_smoothness,x_min,x_max ,vx_min
        ,vx_max ,ax_min ,ax_max ,y_min,y_max ,vy_min ,vy_max,ay_min ,ay_max ,z_min ,z_max ,vz_min ,vz_max ,az_min
         ,az_max, az_fin,t, v_min, v_max, v_ref, psidot_max, psiddot_max, phi_max, a_max, v_des, v_init, obs_1_vx, obs_2_vx, obs_1_vy, obs_2_vy, 
         x_obs_1_init, x_obs_2_init, y_obs_1_init, y_obs_2_init, alphadot_init, rho_obs, rho_occ, rho_ineq, t_update, t_up;

        int num, nvar,num_obs,num_samples, num_occ, x_r_vec, y_r_vec, z_r_vec, mpc_iter;

        Eigen::ArrayXXf lamda_dyn_x, lamda_dyn_y, lamda_dyn_z, A_eq, b_eq_x ,b_eq_y, b_eq_z,cost_smoothness,r_1, r_2,A_fov,s, A_orient, lamda_alpha
        ,A_ineq,B_ineq_x ,B_ineq_y ,B_ineq_z,s_ineq_x,lamda_ineq_x ,s_ineq_y,lamda_ineq_y,s_ineq_z,lamda_ineq_z
        ,z_obs, a_obs, b_obs, c_obs,A_obs,alpha_obs, beta_obs, d_obs, A_occ, alpha_occ, d_occ, x_occ, y_occ, z_occ, a_occ,
        b_occ, c_occ,xddot_guess,zddot_guess,yddot_guess,x_guess, y_guess,z_guess,alpha, beta, gamma,f,x,y,z, xdot, ydot, xddot, yddot, cx,cy, x_obs, y_obs;
  };


 struct TwoVarScalar
  {
    ProbData prob_data;
    float scalar;
  };

struct DroneVelocities
{
  ProbData prob_data; 
  float vx_drone, vy_drone, ax_drone, ay_drone, alphadot_drone, vx_local_drone, vy_local_drone;
};

namespace optim
{
  Eigen::ArrayXXf vstack( const Eigen::ArrayXXf &m1, const Eigen::ArrayXXf &m2);
  Eigen::ArrayXXf hstack( const Eigen::ArrayXXf &m1, const Eigen::ArrayXXf &m2);
  Eigen::ArrayXXf ones(int row, int col);
  Eigen::ArrayXXf clip2(Eigen::ArrayXXf min, Eigen::ArrayXXf max, Eigen::ArrayXXf arr);
  Eigen::ArrayXXf clip(float min, float max, Eigen::ArrayXXf arr);
  Eigen::ArrayXXf diff(Eigen::ArrayXXf arr);
  Eigen::ArrayXXf maximum(float val, Eigen::ArrayXXf arr2);
  Eigen::ArrayXXf minimum(float val, Eigen::ArrayXXf arr2);
  Eigen::ArrayXXf linspace(float t_init, float t_end, int steps);
  Eigen::ArrayXXf arctan2(Eigen::ArrayXXf arr1, Eigen::ArrayXXf arr2);
  Eigen::ArrayXXf cumsum(Eigen::ArrayXXf arr1, Eigen::ArrayXXf arr2);
  // Eigen::ArrayXXf reshape(Eigen::ArrayXXf  arr, int row, int col);
  Eigen::ArrayXXf reshape(Eigen::ArrayXXf x, uint32_t r, uint32_t c);

  ProbData wrapper_fov(ProbData prob_data, Eigen::ArrayXXf P, Eigen::ArrayXXf Pdot, Eigen::ArrayXXf Pddot, Eigen::ArrayXXf u, float delt_factor);

  ThreeVar bernstein_coeff_order10_new(float n, float tmin, float tmax, Eigen::ArrayXXf t_actual);
  float binomialCoeff(float n, float k);
  DroneVelocities main_fov(ProbData prob_data, ProbData sim_data, int sim_index, int maxiter, float delt_factor,
  Eigen::ArrayXXf P_up, Eigen::ArrayXXf Pdot_up, Eigen::ArrayXXf Pddot_up, float t_update, float t_up,
  Eigen::ArrayXXf P, Eigen::ArrayXXf Pdot, Eigen::ArrayXXf Pddot, Eigen::ArrayXXf tot_time, Eigen::ArrayXXf u);
  TwoVarScalar compute_x (ProbData prob_data, Eigen::ArrayXXf P, Eigen::ArrayXXf Pdot,
  Eigen::ArrayXXf Pddot, float rho_dyn, float rho_fov, float rho_ineq,
  float rho_obs, float rho_occ, float rho_fov_z, Eigen::ArrayXXf u);
  

  ProbData compute_init_guess(ProbData prob_data, Eigen::ArrayXXf P, Eigen::ArrayXXf Pdot, Eigen::ArrayXXf Pddot);
  Eigen::ArrayXXf compute_A_occ(Eigen::ArrayXXf P, ProbData prob_data, Eigen::ArrayXXf u);
  Eigen::ArrayXXf addNewAxis(Eigen::ArrayXXf array, int new_axis_dimension);

  /*Data compute_init_guess(Data prob_data, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot);
  Data wrapper_fov(Data prob_data, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot);
  Data compute_orient_traj(Data prob_data, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, ArrayXXf alpha_temp, float rho_orient);
  Data compute_w(Data prob_data, float rho_fov, float rho_w);
  two_var compute_x(Data prob_data, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, float rho_dyn, float rho_fov, float rho_ineq);
  */
}
#endif