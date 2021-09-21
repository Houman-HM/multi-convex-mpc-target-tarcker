/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


/*
IMPORTANT: This file should serve as a starting point to develop the user
code for the OCP solver. The code below is for illustration purposes. Most
likely you will not get good results if you execute this code without any
modification(s).

Please read the examples in order to understand how to write user code how
to run the OCP solver. You can find more info on the website:
www.acadotoolkit.org

*/

#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <stdlib.h>
#include <stdio.h>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */
#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */
#define N           ACADO_N   /* Number of intervals in the horizon. */
#define MPC 		0
#define VERBOSE		1 - MPC
#define pi M_PI


int NUM_STEPS = 10; // solver iterations

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

void plot(double x_init, double y_init);
double calc_dist(double x1, double y1, double x2, double y2);
void init_state(double x, double y, double vx, double vy, double ax, double ay, int flag);

void init_weights(double wobs, double wvx, double wvy, double wax, double way, double wo);
void init_OD(double xm, double ym, double x0, double y0, double r0, double x1, double y1, double r1, double x2, double y2, double r2);
void fov_circle(double M, double rho, double xm, double ym, double x, double y);
double vis_score(double x, double y, double xm, double ym, double xo, double yo);

// change these variables
double xo[] = {1.2, -3.0, 100.0}, 			// initial x-position of three obstacles 
	   yo[] = {3.0, 1.0, 100.0},				// initial y-position of three obstacles
	   velx[] = {-0.0, 0.0, 0.0},			// initial x-velocity of three obstacles
	   vely[] = {-0.0, -0.0, 0.0},			// initial y-velocity of three obstacles
	   ro[] = {0.6, 0.6, 0.6},				// radius of three obstacles
	   WO = 100,							// occlusion cost weight
	   WOBS = 70,							// obstacle avoidance weight
	   WV = 50, WA = 1,						// weights of velocity and acceleration term (when occlusion-free state only then velocity term is invoked)
	   X0 = 0.0, Y0 = 0.0,						// initial x,y position of quadrotor
	   XM = -1.0, YM = 5.0;						// marker x,y position

double  dt = 0.1;							// planning resolution	

FILE *fp0, *fp1, *fp2, *fp3, *fp4, *fp5, *fp6, *fp7, *fp8, *fp9, *fp10, *fp11, *fp12;

int main( )
{
	
	int    i, iter;
	acado_timer t, t2;
	acado_initializeSolver();

	// initialize controls
	for (i = 0; i < N; ++i)  
	{
		acadoVariables.u[ i*NU + 0 ] = 0.0;		
		acadoVariables.u[ i*NU + 1 ] = 0.0;		
	}	
	// initialize desired values of each cost term
	for (int i = 0; i < N; ++i)  
	{
		acadoVariables.y[ i*NY + 0 ] = 0.0;		
		acadoVariables.y[ i*NY + 1 ] = 0.0;		
		acadoVariables.y[ i*NY + 2 ] = 0.0;		
		acadoVariables.y[ i*NY + 3 ] = 0.0;		
		acadoVariables.y[ i*NY + 4 ] = 0.0;		
		acadoVariables.y[ i*NY + 5 ] = 0.0;			
	}
	acadoVariables.yN[ 0 ] = 5.0;
	acadoVariables.yN[ 1 ] = 5.0;
	acadoVariables.yN[ 2 ] = 0.0;
	acadoVariables.yN[ 3 ] = 0.0;
	acadoVariables.yN[ 4 ] = 0.0;
	acadoVariables.yN[ 5 ] = 0.0;

	double xm = XM, x_init = X0,
		   ym = YM, y_init = Y0,
		   wax = WA, way = WA, 
		   wo = WO, wobs = WOBS, wv = WV;
	double yaw, yawRate = 0.0;
	
	// initialize state of quadrotor
	init_state(x_init, y_init, 0.0, 0.0, 0.0, 0.0, 1.0);
	
	// intialize online data variables
	init_OD(xm, ym, xo[0], yo[0], ro[0], xo[1], yo[1], ro[1], xo[2], yo[2], ro[2]);
	
	// initialize weights 
	init_weights(wobs, 0.0, 0.0, wax, way, wo);
	
	acado_preparationStep();

	double time = 0;
	int loop = -1, cnt = 0;
	acado_tic( &t2 );

	if(MPC)
	{
		fp2 = fopen("x.txt","w");
		fp3 = fopen("y.txt","w");
		fp4 = fopen("obsx0.txt","w");
		fp5 = fopen("obsy0.txt","w");
		fp6 = fopen("obsx1.txt","w");
		fp7 = fopen("obsy1.txt","w");
		fp8 = fopen("obsx2.txt","w");
		fp9 = fopen("obsy2.txt","w");

		fp0 = fopen("obs0.txt","w");
		fp1 = fopen("obs1.txt","w");
		fp12 = fopen("obs2.txt","w");
		fp10 = fopen("info.txt","w");
		fp11 = fopen("details.txt","w");
		

		yaw = atan2(ym - acadoVariables.x0[0], xm - acadoVariables.x0[0]);
		fprintf(fp11, "%f %f %f %f %f %f %f %f %f\n", acadoVariables.x0[0], acadoVariables.x0[1], acadoVariables.x0[2], acadoVariables.x0[3], acadoVariables.x0[4], acadoVariables.x0[5], yaw, 0.0, 0.0);
	}
	do
	{
		double x = acadoVariables.x0[0], 
			   y = acadoVariables.x0[1],
			   vx = acadoVariables.x0[2],
			   vy = acadoVariables.x0[3],
			   ax = acadoVariables.x0[4],
			   ay = acadoVariables.x0[5];
		
		acado_tic( &t );
		for(iter = 0; iter < NUM_STEPS; ++iter)
		{
			acado_feedbackStep( );
			x = acadoVariables.x[NX + 0];
			y = acadoVariables.x[NX + 1];
			vx = acadoVariables.x[NX + 2];
			vy = acadoVariables.x[NX + 3];
			ax = acadoVariables.x[NX + 4];
			ay = acadoVariables.x[NX + 5];
			
			if(NUM_STEPS == 1)
			{
				acado_shiftStates(2, 0, 0);
        		acado_shiftControls( 0 );
			}
			if(VERBOSE)
			{			
				printf("Real-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );
				printf("XY %f %f\n", acadoVariables.x[(N)*NX + 0], acadoVariables.x[(N)*NX + 1]);	
			}
			acado_preparationStep();				
		}
		real_t te = acado_toc( &t );

		if(loop >= 0)
		{
			time = time + te;
			loop++;
		}
		else
		{
			// fprintf(fp10, "Warm Up time = %f \n", te);
			loop++;
			//printf("Warm up %f \n", te);
		}		
		NUM_STEPS = 1;		
		if(!MPC)
		{
			fp2 = fopen("x.txt","w");
			fp3 = fopen("y.txt","w");

			for (int i = 1; i < (N + 1); i++) 
			{
			
				double x = acadoVariables.x[NX*i + 0];
				double y = acadoVariables.x[NX*i + 1];
				fprintf(fp2,"%f \n", x);
				fprintf(fp3,"%f \n", y);
			}

			goto jump;
		}
			
		
		double a = sqrt(ax*ax + ay*ay);

		yawRate = (atan2(ym - y, xm - x) - yaw) / dt;
		yaw = atan2(ym - y, xm - x);
		init_state(x, y, vx, vy, ax, ay, 0.0);
		
		double score_0 = vis_score(x, y, xm, ym, xo[0], yo[0]);
		double score_1 = vis_score(x, y, xm, ym, xo[1], yo[1]);
		double score_2 = vis_score(x, y, xm, ym, xo[2], yo[2]);

		
		fprintf(fp2,"%f \n", x);
		fprintf(fp3,"%f \n", y);
		fprintf(fp0,"%f %f %f %f \n", xo[0], yo[0], velx[0], vely[0]);
		fprintf(fp1,"%f %f %f %f \n", xo[1], yo[1], velx[1], vely[1]);
		fprintf(fp12,"%f %f %f %f \n", xo[2], yo[2], velx[2], vely[2]);
		fprintf(fp4,"%f \n", xo[0]);
		fprintf(fp5,"%f \n", yo[0]);
		fprintf(fp6,"%f \n", xo[1]);
		fprintf(fp7,"%f \n", yo[1]);
		fprintf(fp8,"%f \n", xo[2]);
		fprintf(fp9,"%f \n", yo[2]);
		fprintf(fp11, "%f %f %f %f %f %f %f %f %f \n", x, y, vx, vy, ax, ay, yaw, yawRate, te);
		
		double dist0 = calc_dist(x, y, xo[0], yo[0]);
		double dist1 = calc_dist(x, y, xo[1], yo[1]);
		double dist2 = calc_dist(x, y, xo[2], yo[2]);


		if(dist0 <= ro[0])
			printf("Collision with obs0_____________________________________ = %f %f\n", dist0, wobs * (1/dist0));
		else if(dist1 <= ro[1])
			printf("Collision with obs1_____________________________________ = %f %f\n", dist1,  wobs * (1/dist1));	
		else if(dist2<= ro[2])
			printf("Collision with obs2_____________________________________ = %f %f\n", dist2,  wobs * (1/dist2));	

		yo[0] += vely[0] * dt;
		xo[0] += velx[0] * dt;
		yo[1] += vely[1] * dt;
		xo[1] += velx[1] * dt;
		yo[2] += vely[2] * dt;
		xo[2] += velx[2] * dt;

		init_OD(xm, ym, xo[0], yo[0], ro[0], xo[1], yo[1], ro[1], xo[2], yo[2], ro[2]);		

		if(fabs(vx) <= 0.001 && fabs(vy) <= 0.001)
			break;
		real_t te2 = acado_toc( &t2 );
		
		if(te2 >= 35)
		{
			break;	
		}
		if(score_0 <= 0.0001 && score_1 <= 0.0001 && score_2 <= 0.0001) // occlusion free: activate velocity term so that quadrotor comes to rest
			init_weights(wobs, wv, wv, wax, way, wo);		
	}
	while(MPC);
	fclose(fp0);
	fclose(fp1);
	fclose(fp2);
	fclose(fp3);
	fclose(fp4);
	fclose(fp5);
	fclose(fp6);
	fclose(fp7);
	fclose(fp12);
	fclose(fp8);
	fclose(fp9);
	fprintf(fp10, "Loop Time = %f \n", time/loop);
	fclose(fp10);
	fclose(fp11);
	jump:;

	printf("Average Time per iteration %f \n", time / loop);
	
    return 0;
}
double vis_score(double x, double y, double xm, double ym, double xo, double yo)
{
	double proj = (xm - x) * (xo - x) + (ym - y) * (yo - y);
	double rh = (xm - x) * (xm - x) + (ym - y) * (ym - y);
	double rt = (xo - x) * (xo - x) + (yo - y) * (yo - y); 		
	double dv = proj / rh;  
    double temp = proj - rt + 1;
	
	return  dv * (sqrt(pow(dv, 2)) / (dv + 0.000000000001) + 1) * (sqrt(pow(temp, 2)) / (temp + 0.000000000001) + 1);	
}
void init_OD(double xm, double ym, double x0, double y0, double r0, double x1, double y1, double r1, double x2, double y2, double r2)
{
	if(!MPC)
	{
		fp4 = fopen("obsx0.txt","w");
		fp5 = fopen("obsy0.txt","w");
		fp6 = fopen("obsx1.txt","w");
		fp7 = fopen("obsy1.txt","w");
		fp8 = fopen("obsx2.txt","w");
		fp9 = fopen("obsy2.txt","w");
	}
	for (int i = 0; i < (N + 1); ++i)
	{
	  acadoVariables.od[i * NOD + 0] = xm;
	  acadoVariables.od[i * NOD + 1] = ym;
	  
	  acadoVariables.od[i * NOD + 2] = x0;
	  acadoVariables.od[i * NOD + 3] = y0;
	  acadoVariables.od[i * NOD + 4] = x1;
	  acadoVariables.od[i * NOD + 5] = y1;
	  

	  if(!MPC)
	  {
		
		fprintf(fp4,"%f \n", x0);
		fprintf(fp5,"%f \n", y0);
		fprintf(fp6,"%f \n", x1);
		fprintf(fp7,"%f \n", y1);
	  }
	  y0 += vely[0] * dt;
	  x0 += velx[0] * dt;

	  y1 += vely[1] * dt;
	  x1 += velx[1] * dt;

	  y2 += vely[2] * dt;
	  x2 += velx[2] * dt;
	}
	if(!MPC)
	{	
		fclose(fp4);
		fclose(fp5);
		fclose(fp6);
		fclose(fp7);
		fclose(fp8);
		fclose(fp9);
	}
}
void init_weights(double wobs, double wvx, double wvy, double wax, double way, double wo)
{
	  for (int i = 0; i < N; i++)
	  {
			acadoVariables.W[NY*NY*i + (NY+1)*0] = 500; 				// smoothness
    		acadoVariables.W[NY*NY*i + (NY+1)*1] = 500;					// smoothness
			acadoVariables.W[NY*NY*i + (NY+1)*2] = 1;//1100;			// obs0
			acadoVariables.W[NY*NY*i + (NY+1)*3] = 1;//1100;			// obs1
    		acadoVariables.W[NY*NY*i + (NY+1)*4] = 1000;				// occ0
    		acadoVariables.W[NY*NY*i + (NY+1)*5] = 1000;				// occ1
	  }
	  acadoVariables.WN[ (NY+1)*0 ] = 10000;				// x
	  acadoVariables.WN[ (NY+1)*1 ] = 10000;				// y
	  acadoVariables.WN[ (NY+1)*2 ] = 10000;				// vx
	  acadoVariables.WN[ (NY+1)*3 ] = 10000;				// vy
	  acadoVariables.WN[ (NY+1)*4 ] = 10000;				// ax
	  acadoVariables.WN[ (NY+1)*5 ] = 10000;				// ay
}


void init_state(double x, double y, double vx, double vy, double ax, double ay, int flag)
{
	acadoVariables.x0[ 0 ] = x;
	acadoVariables.x0[ 1 ] = y;
	acadoVariables.x0[ 2 ] = vx;
	acadoVariables.x0[ 3 ] = vy;
	acadoVariables.x0[ 4 ] = ax;
	acadoVariables.x0[ 5 ] = ay;

	if(flag == 1)
	{	
		acadoVariables.x[ 0 ] = x;
		acadoVariables.x[ 1 ] = y;
		acadoVariables.x[ 2 ] = vx;
		acadoVariables.x[ 3 ] = vy;
		acadoVariables.x[ 4 ] = ax;
		acadoVariables.x[ 5 ] = ay;
		for (int i = 1; i < (N + 1); i++)  
		{
			acadoVariables.x[ i*NX + 2 ] = acadoVariables.x[ (i - 1)*NX + 2 ];
			acadoVariables.x[ i*NX + 1 ] = acadoVariables.x[ (i - 1)*NX + 1 ];
			acadoVariables.x[ i*NX + 0 ] = acadoVariables.x[ (i - 1)*NX + 0 ];
			acadoVariables.x[ i*NX + 3 ] = acadoVariables.x[ (i - 1)*NX + 3 ];
			acadoVariables.x[ i*NX + 4 ] = acadoVariables.x[ (i - 1)*NX + 4 ];
			acadoVariables.x[ i*NX + 5 ] = acadoVariables.x[ (i - 1)*NX + 5 ];
		} 
	}
}

double calc_dist(double x1, double y1, double x2, double y2)
{
	return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2)); 
}


