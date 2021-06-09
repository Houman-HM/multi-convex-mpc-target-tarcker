#include "target_tracker/tracker/fov_tracker_6_obs.h"
#include <chrono>
#include <iostream>
#include <ctime>
#include <ratio>
#include <iostream>
#include <fstream>
#include <eigen-quadprog/QuadProg.h>
#include <eigen-quadprog/eigen_quadprog_api.h>
using namespace Eigen;
using namespace std::chrono;


Eigen::ArrayXXf  unwrap(Eigen::ArrayXXf in, Eigen::ArrayXXf out, int len) {
    out(0) = in(0);
    for (int i = 1; i < len; i++)
     {
        float d = in(i) - in(i-1);
        d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
        out(i) = out(i-1) + d;
    }
    return out;
}


Eigen::ArrayXXf optim::vstack(const Eigen::ArrayXXf &m1,const Eigen::ArrayXXf &m2)
{

	if(m1.rows() == 0){
		return m2;
	}
	else if(m2.rows() == 0){
		return m1;
	}

	uint32_t ncol = m1.cols();
	if(ncol == 0){
		ncol = m2.cols();
	}

	Eigen::ArrayXXf rm(m1.rows()+m2.rows(), ncol);
	rm << m1, m2;

	return rm;
}

ArrayXXf optim::diff(ArrayXXf arr)
	{
	    ArrayXXf temp(arr.rows() - 1, 1);
	    for (int i = 0; i < arr.rows() - 1; i++)
	    {
		temp(i) = arr(i + 1) - arr(i);
	    }
	    return temp;
	}

Eigen::ArrayXXf optim::hstack( const Eigen::ArrayXXf &m1, const Eigen::ArrayXXf &m2)
{

	if(m1.cols() == 0){
		return m2;
	}
	else if(m2.cols() == 0){
		return m1;
	}

	uint32_t nrow = m1.rows();
	if(nrow == 0){
		nrow = m2.rows();
	}

	Eigen::ArrayXXf rm(nrow, m1.cols()+m2.cols());
	rm << m1, m2;

	return rm;
}

ArrayXXf optim::maximum(float val, ArrayXXf arr2)
	{
		ArrayXXf temp(arr2.rows(), arr2.cols());
		temp = val;

		int k = 0;
		for (int i = 0; i < arr2.cols(); i++)
		{
			for (int j = 0; j < arr2.rows(); j++)
			{
				if (arr2(k) > val)
					temp(k) = arr2(k);
				k++;
			}
		}
		return temp;
  }

ArrayXXf optim::arctan2(ArrayXXf arr1, ArrayXXf arr2)
	{
	    ArrayXXf temp(arr1.rows(), arr1.cols());

	    int k = 0;
	    for (int i = 0; i < arr1.cols(); i++)
	    {
		for (int j = 0; j < arr1.rows(); j++)
		{
		    temp(k) = atan2(arr1(k), arr2(k));
		    k++;
		}
	    }
	    return temp;
	}
Eigen::ArrayXXf optim::reshape(Eigen::ArrayXXf x, uint32_t r, uint32_t c)
{
  Eigen::Map<Eigen::ArrayXXf> rx(x.data(), r, c);
  return rx;
}

Eigen::ArrayXXf optim::linspace(float t_init, float t_end, int steps)
{
  return Eigen::VectorXf::LinSpaced(steps, t_init, t_end).array();
}

float optim::binomialCoeff(float n, float k)
{
  if (k == 0 || k == n)
      return 1;

  return binomialCoeff(n - 1, k - 1) +
      binomialCoeff(n - 1, k);
}

Eigen::ArrayXXf optim::addNewAxis(Eigen::ArrayXXf array, int new_axis_dimension)
{
   ArrayXXf result = ArrayXXf::Ones(array.cols(), new_axis_dimension);
   ArrayXXf temp = array.transpose();
    for (int i  = 0; i < result.rows(); i++)
    {
      result.row(i) = temp.row(i);
    }
  return result;
}


Eigen::ArrayXXf optim::clip(float min, float max, ArrayXXf arr)
	{
		for (int k = 0; k < arr.cols() * arr.rows(); k++)
		{
			if (arr(k) > max)
				arr(k) = max;
			if (arr(k) < min)
				arr(k) = min;
		}
		return arr;
	}
ThreeVar optim::bernstein_coeff_order10_new(float n, float tmin, float tmax, Eigen::ArrayXXf t_actual)
{
  float l = tmax - tmin;

	Eigen::ArrayXXf t= (t_actual - tmin)/l;
  Eigen::ArrayXXf P0= binomialCoeff(n, 0) * pow(1.0 - t, n - 0) * pow(t, 0);

  Eigen::ArrayXXf P1 = binomialCoeff(n, 1) * pow(1.0 - t, n - 1) * pow(t, 1.0);

  Eigen::ArrayXXf P2 = binomialCoeff(n, 2) * pow(1.0 - t, n - 2) * pow(t, 2);

  Eigen::ArrayXXf P3 = binomialCoeff(n, 3) * pow(1.0 - t, n - 3) * pow(t, 3);

  Eigen::ArrayXXf P4 = binomialCoeff(n, 4) * pow(1.0 - t, n - 4) * pow(t, 4);

  Eigen::ArrayXXf P5 = binomialCoeff(n, 5) * pow(1.0 - t, n - 5) * pow(t, 5);

  Eigen::ArrayXXf P6 = binomialCoeff(n, 6) * pow(1.0 - t, n - 6) * pow(t, 6);

  Eigen::ArrayXXf P7 = binomialCoeff(n, 7) * pow(1.0 - t, n - 7) * pow(t, 7);

  Eigen::ArrayXXf P8 = binomialCoeff(n, 8) * pow(1.0 - t, n - 8) * pow(t, 8);

  Eigen::ArrayXXf P9 = binomialCoeff(n, 9) * pow(1.0 - t, n - 9) * pow(t, 9);

  Eigen::ArrayXXf P10 = binomialCoeff(n, 10) * pow(1.0 - t, n - 10) * pow(t, 10);

  Eigen::ArrayXXf P0dot = -10.0 * pow(-t + 1.0, 9);

  Eigen::ArrayXXf P1dot = -90.0 * t * pow(-t + 1.0, 8) + 10.0 * pow(-t + 1.0, 9);

  Eigen::ArrayXXf P2dot = -360.0 * pow(t, 2) * pow(-t + 1.0, 7) + 90.0 * t * pow(-t + 1.0, 8);

  Eigen::ArrayXXf P3dot = -840.0 * pow(t, 3) * pow(-t + 1.0, 6) + 360.0 * pow(t, 2) * pow(-t + 1.0, 7);

  Eigen::ArrayXXf P4dot = -1260.0 * pow(t, 4) * pow(-t + 1.0, 5) + 840.0 * pow(t, 3) * pow(-t + 1.0, 6);

  Eigen::ArrayXXf P5dot = -1260.0 * pow(t, 5) * pow(-t + 1.0, 4) + 1260.0 * pow(t, 4) * pow(-t + 1.0, 5);

  Eigen::ArrayXXf P6dot = -840.0 * pow(t, 6) * pow(-t + 1.0, 3) + 1260.0 * pow(t, 5) * pow(-t + 1.0, 4);

  Eigen::ArrayXXf P7dot = -360.0 * pow(t, 7) * pow(-t + 1.0, 2) + 840.0 * pow(t, 6) * pow(-t + 1.0, 3);

  Eigen::ArrayXXf P8dot = 45.0 * pow(t, 8) * (2.0 * t - 2.0) + 360.0 * pow(t, 7) * pow(-t + 1.0, 2);

  Eigen::ArrayXXf P9dot = -10.0 * pow(t, 9) + 9.0 * pow(t, 8) * (-10.0 * t + 10.0);

  Eigen::ArrayXXf P10dot = 10.0 * pow(t, 9);


  Eigen::ArrayXXf P0ddot = 90.0 * pow(-t + 1.0, 8.0);

  Eigen::ArrayXXf P1ddot = 720.0 * t * pow(-t + 1.0, 7) - 180.0 * pow(-t + 1.0, 8);

  Eigen::ArrayXXf P2ddot = 2520.0 * pow(t, 2) * pow(-t + 1.0, 6) - 1440.0 * t * pow(-t + 1.0, 7) + 90.0 * pow(-t + 1.0, 8);

  Eigen::ArrayXXf P3ddot = 5040.0 * pow(t, 3) * pow(-t + 1.0, 5) - 5040.0 * pow(t, 2) * pow(-t + 1.0, 6) + 720.0 * t * pow(-t + 1.0, 7);

  Eigen::ArrayXXf P4ddot = 6300.0 * pow(t, 4) * pow(-t + 1.0, 4) - 10080.0 * pow(t, 3) * pow(-t + 1.0, 5) + 2520.0 * pow(t, 2) * pow(-t + 1.0, 6);

  Eigen::ArrayXXf P5ddot = 5040.0 * pow(t, 5) * pow(-t + 1.0, 3) - 12600.0 * pow(t, 4) * pow(-t + 1.0, 4) + 5040.0 * pow(t, 3) * pow(-t + 1.0, 5);

  Eigen::ArrayXXf P6ddot = 2520.0 * pow(t, 6) * pow(-t + 1.0, 2) - 10080.0 * pow(t, 5) * pow(-t + 1.0, 3) + 6300.0 * pow(t, 4) * pow(-t + 1.0, 4);

  Eigen::ArrayXXf P7ddot = -360.0 * pow(t, 7) * (2.0 * t - 2.0) - 5040.0 * pow(t, 6) * pow(-t + 1.0, 2) + 5040.0 * pow(t, 5) * pow(-t + 1.0, 3);

  Eigen::ArrayXXf P8ddot = 90.0 * pow(t, 8) + 720.0 * pow(t, 7) * (2.0 * t - 2.0) + 2520.0 * pow(t, 6) * pow(-t + 1.0, 2);

  Eigen::ArrayXXf P9ddot = -180.0 * pow(t, 8) + 72.0 * pow(t, 7) * (-10.0 * t + 10.0);

  Eigen::ArrayXXf P10ddot = 90.0 * pow(t, 8);


	Eigen::ArrayXXf	P = hstack(hstack(hstack(hstack(hstack(hstack(hstack(hstack(hstack(hstack(P0, P1), P2), P3), P4), P5), P6), P7), P8), P9), P10);
		
	Eigen::ArrayXXf	Pdot = hstack(hstack(hstack(hstack(hstack(hstack(hstack(hstack(hstack(hstack(P0dot, P1dot), P2dot), P3dot), P4dot), P5dot), 
																P6dot), P7dot), P8dot), P9dot), P10dot)/l;

	Eigen::ArrayXXf	Pddot = hstack(hstack(hstack(hstack(hstack(hstack(hstack(hstack(hstack(hstack(P0ddot, P1ddot), P2ddot), P3ddot), P4ddot), P5ddot),
	 															P6ddot), P7ddot), P8ddot), P9ddot), P10ddot)/(pow(l,2));


  ThreeVar temp;
  temp.first = P;
  temp.second = Pdot;
  temp.third = Pddot;
  return temp;
  
}

Eigen::ArrayXXf optim::compute_A_occ(Eigen::ArrayXXf P, ProbData prob_data, Eigen::ArrayXXf u)
{
  // std::cout << "In A_OCC" << std::endl;
  Eigen::ArrayXXf temp_mat_1, temp, A_occ;
  temp_mat_1 = (1 - u(0)) * P;
  for (int i = 1; i < prob_data.num_samples; i++)
  {
    temp = (1 - u(i)) * P;
    temp_mat_1 = vstack(temp_mat_1, temp);
    A_occ = vstack(vstack(vstack(vstack(vstack(temp_mat_1, temp_mat_1), temp_mat_1), temp_mat_1), temp_mat_1), temp_mat_1);
  }
  // std::cout << "Out of A_OCC" << std::endl;
  return A_occ;
}


TwoVarScalar optim::compute_x (ProbData prob_data, ArrayXXf P, ArrayXXf Pdot,
 ArrayXXf Pddot, float rho_fov, ArrayXXf u)
{
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    prob_data.d_fov = reshape(prob_data.d_fov, prob_data.num,1);
    prob_data.alpha_fov = reshape(prob_data.alpha_fov, prob_data.num,1);
  
    ArrayXXf b_fov_x = prob_data.r_1.row(0).transpose() + prob_data.d_fov * cos(prob_data.alpha_fov);
    ArrayXXf b_fov_y = prob_data.r_1.row(1).transpose() + prob_data.d_fov * sin(prob_data.alpha_fov);

    ArrayXXf x_occ_1 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf y_occ_1 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf x_occ_2 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf y_occ_2 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf x_occ_3 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf y_occ_3 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf x_occ_4 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf y_occ_4 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());

    ArrayXXf x_occ_5 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf y_occ_5 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());

    ArrayXXf x_occ_6 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf y_occ_6 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());

    auto u_temp = reshape(u, prob_data.num_samples, 1);

    

    for (size_t i = 0; i < prob_data.num_samples; i++)
    {
      x_occ_1.row(i)=  (prob_data.x_obs.row(0) - prob_data.r_1.row(0) * u_temp(i));
      y_occ_1.row(i)=  (prob_data.y_obs.row(0) - prob_data.r_1.row(1) * u_temp(i));
      x_occ_2.row(i)=  (prob_data.x_obs.row(1) - prob_data.r_1.row(0) * u_temp(i));
      y_occ_2.row(i)=  (prob_data.y_obs.row(1) - prob_data.r_1.row(1) * u_temp(i));
      x_occ_3.row(i)=  (prob_data.x_obs.row(2) - prob_data.r_1.row(0) * u_temp(i));
      y_occ_3.row(i)=  (prob_data.y_obs.row(2) - prob_data.r_1.row(1) * u_temp(i));
      x_occ_4.row(i)=  (prob_data.x_obs.row(3) - prob_data.r_1.row(0) * u_temp(i));
      y_occ_4.row(i)=  (prob_data.y_obs.row(3) - prob_data.r_1.row(1) * u_temp(i));

      x_occ_5.row(i)=  (prob_data.x_obs.row(4) - prob_data.r_1.row(0) * u_temp(i));
      y_occ_5.row(i)=  (prob_data.y_obs.row(4) - prob_data.r_1.row(1) * u_temp(i));

      x_occ_6.row(i)=  (prob_data.x_obs.row(5) - prob_data.r_1.row(0) * u_temp(i));
      y_occ_6.row(i)=  (prob_data.y_obs.row(5) - prob_data.r_1.row(1) * u_temp(i));
    }

    ArrayXXf x_occ = vstack(vstack(vstack(vstack(vstack(x_occ_1, x_occ_2), x_occ_3), x_occ_4), x_occ_5), x_occ_6);
    ArrayXXf y_occ = vstack(vstack(vstack(vstack(vstack(y_occ_1, y_occ_2), y_occ_3), y_occ_4), y_occ_5), y_occ_6);

    ArrayXXf temp_x_occ = (prob_data.d_occ * cos(prob_data.alpha_occ)).colwise() * prob_data.a_occ.col(0);


		ArrayXXf b_x_occ = reshape(x_occ.transpose(), prob_data.num * prob_data.num_occ, 1) + reshape(temp_x_occ.transpose(), prob_data.num * prob_data.num_occ, 1);

    ArrayXXf temp_y_occ = (prob_data.d_occ * sin(prob_data.alpha_occ)).colwise() * prob_data.b_occ.col(0);
    ArrayXXf b_y_occ = reshape(y_occ.transpose(), prob_data.num * prob_data.num_occ, 1) + reshape(temp_y_occ.transpose(), prob_data.num * prob_data.num_occ, 1);

    
		ArrayXXf obj_xy = prob_data.cost_smoothness + rho_fov * (prob_data.A_fov.transpose().matrix() * prob_data.A_fov.matrix()).array() + 
    prob_data.rho_occ * (prob_data.A_occ.transpose().matrix() * prob_data.A_occ.matrix()).array();
    //     std::cout <<"Here" << std::endl;


		ArrayXXf lincost_x = -prob_data.lamda_dyn_x - rho_fov * (prob_data.A_fov.transpose().matrix() * b_fov_x.matrix()).array() -
     prob_data.rho_occ * (prob_data.A_occ.transpose().matrix() * b_x_occ.matrix()).array();

		ArrayXXf lincost_y = -prob_data.lamda_dyn_y - rho_fov * (prob_data.A_fov.transpose().matrix() * b_fov_y.matrix()).array() - 
     prob_data.rho_occ * (prob_data.A_occ.transpose().matrix() * b_y_occ.matrix()).array();

    Eigen::QuadProgDense solver_x(prob_data.nvar,prob_data.A_eq.rows(), prob_data.A_ineq.rows());
		Eigen::QuadProgDense solver_y(prob_data.nvar,prob_data.A_eq.rows(), prob_data.A_ineq.rows());


		solver_x.solve(obj_xy.cast<double>().matrix(),lincost_x.cast<double>().matrix(),prob_data.A_eq.cast<double>().matrix(),prob_data.b_eq_x.cast<double>().matrix(),
		 prob_data.A_ineq.cast<double>().matrix(), prob_data.B_ineq_x.transpose().cast<double>().matrix());
		 
		
		solver_y.solve(obj_xy.cast<double>().matrix(),lincost_y.cast<double>().matrix(),prob_data.A_eq.cast<double>().matrix(),prob_data.b_eq_y.cast<double>().matrix(),
		 prob_data.A_ineq.cast<double>().matrix(), prob_data.B_ineq_y.transpose().cast<double>().matrix());

    ArrayXXf primal_sol_x = solver_x.result().cast<float>();
    ArrayXXf primal_sol_y = solver_y.result().cast<float>();

    prob_data.x = P.matrix() * primal_sol_x.matrix();
    prob_data.y = P.matrix() * primal_sol_y.matrix();

    prob_data.xddot = Pddot.matrix() * primal_sol_x.matrix();
    prob_data.yddot = Pddot.matrix() * primal_sol_y.matrix();

    prob_data.xdot = Pdot.matrix() * primal_sol_x.matrix();
    prob_data.ydot = Pdot.matrix() * primal_sol_y.matrix();

    prob_data.cx = primal_sol_x;
    prob_data.cy = primal_sol_y;

    TwoVarScalar retrun_value;
    retrun_value.prob_data = prob_data;

    return retrun_value;
  }

ProbData optim::wrapper_fov(ProbData prob_data, Eigen::ArrayXXf P, Eigen::ArrayXXf Pdot, Eigen::ArrayXXf Pddot, Eigen::ArrayXXf u, float delt_factor)
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  
  float rho_fov = prob_data.rho_fov;
  ArrayXXf d_fov_check = ArrayXXf::Ones(prob_data.maxiter,1);
  ArrayXXf res_fov = ArrayXXf::Ones(prob_data.maxiter,1);
  ArrayXXf res_occ = ArrayXXf::Ones(1, prob_data.maxiter);
  ArrayXXf d_min_occ = ArrayXXf::Ones(1, prob_data.maxiter);

  for (size_t i = 0; i < prob_data.maxiter; i++)
  {

    TwoVarScalar compute_x_return_value = compute_x( prob_data, P, Pdot, Pddot, rho_fov, u);
    prob_data  = compute_x_return_value.prob_data;

    //////////// FOV Computation

    ArrayXXf wc_alpha_fov = (-prob_data.r_1.row(0)).rowwise() + prob_data.x.transpose().row(0);

    ArrayXXf ws_alpha_fov = (-prob_data.r_1.row(1)).rowwise() + prob_data.y.transpose().row(0);

    prob_data.alpha_fov = arctan2(ws_alpha_fov, wc_alpha_fov);

   
    ArrayXXf c1_d_fov = 1.0 * rho_fov * (pow(cos(prob_data.alpha_fov),2) + pow(sin(prob_data.alpha_fov), 2));

    ArrayXXf c2_d_fov = 1.0 * rho_fov * (wc_alpha_fov * cos(prob_data.alpha_fov) + ws_alpha_fov * sin(prob_data.alpha_fov));

    ArrayXXf d_temp_fov = c2_d_fov / c1_d_fov;
     
    prob_data.d_fov = clip(prob_data.d_fov_min, prob_data.d_fov_max, d_temp_fov);

    float d_fov_check = d_temp_fov.minCoeff();

    ArrayXXf res_x_fov_vec = wc_alpha_fov - (prob_data.d_fov * cos(prob_data.alpha_fov));
     
    ArrayXXf res_y_fov_vec = ws_alpha_fov - (prob_data.d_fov * sin(prob_data.alpha_fov));

    ///////////////////// Occlusion Computation


    ArrayXXf x_tilda_temp = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf y_tilda_temp = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    auto u_temp = reshape(u, prob_data.num_samples, 1);

    for (size_t i = 0; i < prob_data.num_samples; i++)
    {
      x_tilda_temp.row(i)=  prob_data.x.transpose().row(0) - prob_data.x.transpose().row(0) * u_temp(i);
      y_tilda_temp.row(i)= prob_data.y.transpose().row(0) - prob_data.y.transpose().row(0) * u_temp(i);
    }
    ArrayXXf x_tilda = vstack(vstack(vstack(vstack(vstack(x_tilda_temp, x_tilda_temp), x_tilda_temp), x_tilda_temp), x_tilda_temp), x_tilda_temp);
    ArrayXXf y_tilda = vstack(vstack(vstack(vstack(vstack(y_tilda_temp, y_tilda_temp), y_tilda_temp), y_tilda_temp), y_tilda_temp), y_tilda_temp);


    ArrayXXf x_occ_1 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf y_occ_1 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf x_occ_2 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf y_occ_2 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf x_occ_3 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf y_occ_3 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf x_occ_4 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf y_occ_4 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());

    ArrayXXf x_occ_5 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf y_occ_5 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());

    ArrayXXf x_occ_6 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());
    ArrayXXf y_occ_6 = ArrayXXf::Zero(u.rows(), prob_data.x_obs.cols());


    for (size_t i = 0; i < prob_data.num_samples; i++)
    {
      x_occ_1.row(i)=  prob_data.x_obs.row(0) -  prob_data.r_1.row(0) * u_temp(i);
      y_occ_1.row(i)=  (prob_data.y_obs.row(0) - prob_data.r_1.row(1) * u_temp(i));
      x_occ_2.row(i)=  (prob_data.x_obs.row(1) - prob_data.r_1.row(0) * u_temp(i));
      y_occ_2.row(i)=  (prob_data.y_obs.row(1) - prob_data.r_1.row(1) * u_temp(i));
      x_occ_3.row(i)=  (prob_data.x_obs.row(2) - prob_data.r_1.row(0) * u_temp(i));
      y_occ_3.row(i)=  (prob_data.y_obs.row(2) - prob_data.r_1.row(1) * u_temp(i));

      x_occ_4.row(i)=  (prob_data.x_obs.row(3) - prob_data.r_1.row(0) * u_temp(i));
      y_occ_4.row(i)=  (prob_data.y_obs.row(3) - prob_data.r_1.row(1) * u_temp(i));

      x_occ_5.row(i)=  (prob_data.x_obs.row(4) - prob_data.r_1.row(0) * u_temp(i));
      y_occ_5.row(i)=  (prob_data.y_obs.row(4) - prob_data.r_1.row(1) * u_temp(i));

      x_occ_6.row(i)=  (prob_data.x_obs.row(5) - prob_data.r_1.row(0) * u_temp(i));
      y_occ_6.row(i)=  (prob_data.y_obs.row(5) - prob_data.r_1.row(1) * u_temp(i));
    }

    ArrayXXf x_occ = vstack(vstack(vstack(vstack(vstack(x_occ_1, x_occ_2), x_occ_3), x_occ_4), x_occ_5), x_occ_6);
    ArrayXXf y_occ = vstack(vstack(vstack(vstack(vstack(y_occ_1, y_occ_2), y_occ_3), y_occ_4), y_occ_5), y_occ_6);

    ArrayXXf wc_alpha_occ = (x_tilda-x_occ);
    ArrayXXf ws_alpha_occ = (y_tilda-y_occ);

    prob_data.alpha_occ = arctan2(ws_alpha_occ.colwise() * prob_data.a_occ.col(0), wc_alpha_occ.colwise() * prob_data.b_occ.col(0));

    ArrayXXf c1_d_occ = 1.0 * prob_data.rho_occ * (pow(cos(prob_data.alpha_occ), 2).colwise() * pow(prob_data.a_occ.col(0), 2)
     + pow(sin(prob_data.alpha_occ), 2).colwise() * pow(prob_data.b_occ.col(0),2));

    ArrayXXf c2_d_occ = 1.0 * prob_data.rho_occ * ((wc_alpha_occ * cos(prob_data.alpha_occ)).colwise() * prob_data.a_occ.col(0)
    + (ws_alpha_occ * sin(prob_data.alpha_occ)).colwise() * prob_data.b_occ.col(0));

    ArrayXXf d_temp_occ = c2_d_occ / c1_d_occ; 
    prob_data.d_occ = maximum(1, d_temp_occ);
    d_min_occ(0,i) = d_temp_occ.minCoeff();
     
    ArrayXXf res_x_occ_vec = wc_alpha_occ - (prob_data.d_occ * cos(prob_data.alpha_occ)).colwise() * prob_data.a_occ.col(0);
  
    ArrayXXf res_y_occ_vec = ws_alpha_occ - ((prob_data.d_occ * sin(prob_data.alpha_occ)).colwise()) * prob_data.b_occ.col(0);
    
    prob_data.lamda_dyn_x = prob_data.lamda_dyn_x - rho_fov * (prob_data.A_fov.transpose().matrix() *
     reshape(res_x_fov_vec.transpose(), res_x_fov_vec.rows() * res_x_fov_vec.cols(), 1).matrix()).array() - 
    prob_data.rho_occ * (prob_data.A_occ.transpose().matrix() * reshape(res_x_occ_vec.transpose(), res_x_occ_vec.rows() * res_x_occ_vec.cols(), 1).matrix()).array();

    prob_data.lamda_dyn_y = prob_data.lamda_dyn_y - rho_fov * (prob_data.A_fov.transpose().matrix() * 
    reshape(res_y_fov_vec.transpose(), res_y_fov_vec.rows() * res_y_fov_vec.cols(), 1).matrix()).array() - 
    prob_data.rho_occ * (prob_data.A_occ.transpose().matrix() * reshape(res_y_occ_vec.transpose(), res_y_occ_vec.rows() * res_y_occ_vec.cols(), 1).matrix()).array();

    ArrayXXf res_fov_vec = hstack(res_x_fov_vec, res_y_fov_vec);
    res_fov(0, i) = res_fov_vec.matrix().lpNorm<2>();
    ArrayXXf res_occ_vec = hstack(res_x_occ_vec, res_y_occ_vec);
    res_occ(0,i) = res_occ_vec.matrix().lpNorm<2>();
  }
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  return prob_data;
}

  DroneVelocities optim::main_fov(ProbData prob_data, ProbData sim_data, int sim_index, int maxiter, float delt_factor,
 ArrayXXf P_up, ArrayXXf Pdot_up, ArrayXXf Pddot_up, float t_update, float t_up,
  ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, ArrayXXf tot_time, ArrayXXf u, 
  ArrayXXf tot_time_up)
{
  prob_data.maxiter = maxiter;
  prob_data.x_init = sim_data.x_init;
  prob_data.vx_init = sim_data.vx_init;
  prob_data.ax_init = sim_data.ax_init;
  prob_data.y_init = sim_data.y_init;
  prob_data.vy_init = sim_data.vy_init;
  prob_data.ay_init = sim_data.ay_init;
  prob_data.alpha_init = sim_data.alpha_init;


  prob_data.t = prob_data.t_fin/prob_data.num;
  prob_data.nvar = P.cols();
  prob_data.A_eq = vstack(vstack(vstack(vstack(P.row(0), Pdot.row(0)), Pddot.row(0)), Pdot.row(Pdot.rows()-1)), Pddot.row(Pddot.rows()-1));

  prob_data.b_eq_x = ArrayXXf(5, 1);
  prob_data.b_eq_y = ArrayXXf(5, 1);
  prob_data.b_eq_z = ArrayXXf(5, 1);

  prob_data.b_eq_x << sim_data.x_init, sim_data.vx_init, sim_data.ax_init, sim_data.vx_fin, sim_data.ax_fin;
  prob_data.b_eq_y << sim_data.y_init, sim_data.vy_init, sim_data.ay_init, sim_data.vy_fin, sim_data.ay_fin;
  prob_data.b_eq_z << sim_data.z_init, sim_data.vz_init, sim_data.az_init, sim_data.vz_fin, sim_data.az_fin;

  prob_data.cost_smoothness = prob_data.weight_smoothness * (Pddot.transpose().matrix() * Pddot.matrix()).array();
  
  float vx_target = sim_data.vx_target;
  float vy_target = sim_data.vy_target;

  ArrayXXf x_target = sim_data.x_target_init + vx_target * tot_time;
  ArrayXXf y_target = sim_data.y_target_init + vy_target * tot_time;

  prob_data.r_1 = vstack(x_target.transpose(), y_target.transpose());

  
  prob_data.A_ineq = vstack(vstack(vstack(Pdot, -Pdot), Pddot), -Pddot);

  prob_data.B_ineq_x = hstack(hstack(hstack(prob_data.vx_max * ArrayXXf::Ones(1, prob_data.num), -prob_data.vx_min * ArrayXXf::Ones(1, prob_data.num)),
    prob_data.ax_max * ArrayXXf::Ones(1, prob_data.num)), -prob_data.ax_min * ArrayXXf::Ones(1, prob_data.num));
  
  prob_data.B_ineq_y = hstack(hstack(hstack(prob_data.vy_max * ArrayXXf::Ones(1, prob_data.num), -prob_data.vy_min * ArrayXXf::Ones(1, prob_data.num)),
    prob_data.ay_max * ArrayXXf::Ones(1, prob_data.num)), -prob_data.ay_min * ArrayXXf::Ones(1, prob_data.num));

  float obs_1_vx = sim_data.obs_1_vx;
  float obs_1_vy = sim_data.obs_1_vy;
  float x_obs_1_init = sim_data.x_obs_1_init;
  float y_obs_1_init = sim_data.y_obs_1_init;

  ArrayXXf x_temp_1 = x_obs_1_init + obs_1_vx * tot_time;
  ArrayXXf y_temp_1 = y_obs_1_init + obs_1_vy * tot_time;

  float obs_2_vx = sim_data.obs_2_vx;
  float obs_2_vy = sim_data.obs_2_vy;
  float x_obs_2_init = sim_data.x_obs_2_init;
  float y_obs_2_init = sim_data.y_obs_2_init;

  ArrayXXf x_temp_2 = x_obs_2_init + obs_2_vx * tot_time;
  ArrayXXf y_temp_2 = y_obs_2_init + obs_2_vy * tot_time;

  float obs_3_vx = sim_data.obs_3_vx;
  float obs_3_vy = sim_data.obs_3_vy;
  float x_obs_3_init = sim_data.x_obs_3_init;
  float y_obs_3_init = sim_data.y_obs_3_init;

  ArrayXXf x_temp_3 = x_obs_3_init + obs_3_vx * tot_time;
  ArrayXXf y_temp_3 = y_obs_3_init + obs_3_vy * tot_time;

  float obs_4_vx = sim_data.obs_4_vx;
  float obs_4_vy = sim_data.obs_4_vy;
  float x_obs_4_init = sim_data.x_obs_4_init;
  float y_obs_4_init = sim_data.y_obs_4_init;

  ArrayXXf x_temp_4 = x_obs_4_init + obs_4_vx * tot_time;
  ArrayXXf y_temp_4 = y_obs_4_init + obs_4_vy * tot_time;

  float obs_5_vx = sim_data.obs_5_vx;
  float obs_5_vy = sim_data.obs_5_vy;
  float x_obs_5_init = sim_data.x_obs_5_init;
  float y_obs_5_init = sim_data.y_obs_5_init;

  ArrayXXf x_temp_5 = x_obs_5_init + obs_5_vx * tot_time;
  ArrayXXf y_temp_5 = y_obs_5_init + obs_5_vy * tot_time;


  float obs_6_vx = sim_data.obs_6_vx;
  float obs_6_vy = sim_data.obs_6_vy;
  float x_obs_6_init = sim_data.x_obs_6_init;
  float y_obs_6_init = sim_data.y_obs_6_init;

  ArrayXXf x_temp_6 = x_obs_6_init + obs_6_vx * tot_time;
  ArrayXXf y_temp_6 = y_obs_6_init + obs_6_vy * tot_time;



  prob_data.x_obs = (hstack(hstack(hstack(hstack(hstack(x_temp_1, x_temp_2), x_temp_3), x_temp_4), x_temp_5),x_temp_6)).transpose();
  prob_data.y_obs = (hstack(hstack(hstack(hstack(hstack(y_temp_1, y_temp_2), y_temp_3), y_temp_4), y_temp_5),y_temp_6)).transpose();

  prob_data.a_obs = ArrayXXf(6, 1);
  prob_data.b_obs = ArrayXXf(6, 1);
  prob_data.c_obs = ArrayXXf(6, 1);
 
  prob_data.a_obs << 0.85, 0.85, 0.85, 0.85, 0.85, 0.85;
  prob_data.b_obs << 0.85, 0.85, 0.85, 0.85, 0.85, 0.85;
  

  prob_data.a_occ = vstack(vstack(vstack(vstack(vstack(prob_data.a_obs(0) * ArrayXXf::Ones(prob_data.num_samples,1), prob_data.a_obs(1) * ArrayXXf::Ones(prob_data.num_samples,1)),
                                  prob_data.a_obs(2) * ArrayXXf::Ones(prob_data.num_samples,1)), prob_data.a_obs(3) * ArrayXXf::Ones(prob_data.num_samples,1)),
                                  prob_data.a_obs(4) * ArrayXXf::Ones(prob_data.num_samples,1)), prob_data.a_obs(5) * ArrayXXf::Ones(prob_data.num_samples,1));

  prob_data.b_occ = vstack(vstack(vstack(vstack(vstack(prob_data.b_obs(0) * ArrayXXf::Ones(prob_data.num_samples,1), prob_data.b_obs(1) * ArrayXXf::Ones(prob_data.num_samples,1)),
                                  prob_data.b_obs(2) * ArrayXXf::Ones(prob_data.num_samples,1)), prob_data.b_obs(3) * ArrayXXf::Ones(prob_data.num_samples,1)),
                                  prob_data.b_obs(4) * ArrayXXf::Ones(prob_data.num_samples,1)), prob_data.b_obs(5) * ArrayXXf::Ones(prob_data.num_samples,1));

  prob_data = wrapper_fov(prob_data, P, Pdot, Pddot, u, delt_factor);

  ArrayXXf x_up = P_up.matrix() * prob_data.cx.matrix();
  ArrayXXf y_up = P_up.matrix() * prob_data.cy.matrix();
  ArrayXXf xddot_up = Pddot_up.matrix() * prob_data.cx.matrix();
  ArrayXXf yddot_up = Pddot_up.matrix() * prob_data.cy.matrix();
  ArrayXXf xdot_up = Pdot_up.matrix() * prob_data.cx.matrix();
  ArrayXXf ydot_up = Pdot_up.matrix() * prob_data.cy.matrix();
  ArrayXXf x_target_up = sim_data.x_target_init + vx_target * tot_time_up;
  ArrayXXf y_target_up = sim_data.y_target_init + vx_target * tot_time_up;
  ArrayXXf alpha_drone_temp = arctan2(y_target_up-y_up, x_target_up-x_up);
  ArrayXXf alpha_drone_temp_stack = sim_data.alpha_init * ArrayXXf::Ones(1,1);
  ArrayXXf alpha_drone_stacked = vstack(alpha_drone_temp_stack,alpha_drone_temp );
  ArrayXXf alpha_drone_unwrapped = unwrap(alpha_drone_stacked, alpha_drone_stacked, alpha_drone_stacked.rows());
  ArrayXXf alpha_drone = alpha_drone_unwrapped.bottomRows(alpha_drone_unwrapped.rows()-1); 
 
  float ax_drone = xddot_up.topRows(10).mean();
  float ay_drone = yddot_up.topRows(10).mean();

  float vx_drone = xdot_up.topRows(10).mean();
  float vy_drone = ydot_up.topRows(10).mean();

  ArrayXXf vx_local = xdot_up * cos(alpha_drone) + ydot_up * sin(alpha_drone);
  ArrayXXf vy_local = -xdot_up * sin(alpha_drone) + ydot_up * cos(alpha_drone);

  float vx_local_drone = vx_local.topRows(10).mean();
  float vy_local_drone = vy_local.topRows(10).mean();
  
  ArrayXXf alpha_init_array = sim_data.alpha_init * ArrayXXf::Ones(1,1);
  ArrayXXf alphadot = diff(vstack(alpha_init_array, alpha_drone)) / t_up;
  float alphadot_drone = alphadot.topRows(10).mean();

  DroneVelocities drone_velocities;

  drone_velocities.prob_data = prob_data;
  drone_velocities.vx_drone = vx_drone;
  drone_velocities.vy_drone = vy_drone;
  drone_velocities.ax_drone = ax_drone;
  drone_velocities.ay_drone = ay_drone;
  drone_velocities.alphadot_drone = alphadot_drone;
  drone_velocities.vx_local_drone = vx_local_drone;
  drone_velocities.vy_local_drone = vy_local_drone;

  return drone_velocities;
}