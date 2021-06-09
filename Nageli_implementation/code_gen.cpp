#include <acado_code_generation.hpp>
#include <iostream>


using namespace std;
int main( )
{
	USING_NAMESPACE_ACADO
	
	// marker
	OnlineData xm;									
	OnlineData ym;									
	
	// obstacles 
	OnlineData xo_1;								
	OnlineData yo_1; 			
	OnlineData ro_1;	
	OnlineData xo_2;				
	OnlineData yo_2;
	OnlineData ro_2;				
	OnlineData xo_3;
	OnlineData yo_3;
	OnlineData ro_3;

	// states
	DifferentialState   x;     
	DifferentialState   y;           
	DifferentialState  vx;
	DifferentialState  vy;	
	DifferentialState  ax;
	DifferentialState  ay;

	// controls
	Control jx;       	  
	Control jy; 	
	
	DifferentialEquation f; 

	f << dot( x ) == vx;
	f << dot( y ) == vy;
	f << dot( vx ) == ax;
	f << dot( vy ) == ay;	
	f << dot( ax ) == jx;
	f << dot( ay ) == jy;

	Function h, hN;
	
	Expression proj_1 = (xm - x) * (xo_1 - x) + (ym - y) * (yo_1 - y);
	Expression proj_2 = (xm - x) * (xo_2 - x) + (ym - y) * (yo_2 - y);
	Expression proj_3 = (xm - x) * (xo_3 - x) + (ym - y) * (yo_3 - y);

	Expression rh = (xm - x) * (xm - x) + (ym - y) * (ym - y);
	
	Expression rt_1 = (xo_1 - x) * (xo_1 - x) + (yo_1 - y) * (yo_1 - y); 		
	Expression rt_2 = (xo_2 - x) * (xo_2 - x) + (yo_2 - y) * (yo_2 - y);
	Expression rt_3 = (xo_3 - x) * (xo_3 - x) + (yo_3 - y) * (yo_3 - y);

	Expression dist_1 = ro_1 + 1.5 - sqrt(rt_1);
	Expression dist_2 = ro_2 + 1.5 - sqrt(rt_2);
	Expression dist_3 = ro_3 + 1.5 - sqrt(rt_3);

	Expression dv_1 = proj_1 / rh;
	Expression dv_2 = proj_2 / rh;
  	Expression dv_3 = proj_3 / rh;

    Expression temp_1 = proj_1 - rt_1 + 1;
	Expression temp_2 = proj_2 - rt_2 + 1;
	Expression temp_3 = proj_3 - rt_3 + 1;

	h << (dist_1 / (sqrt(pow(dist_1, 2)) + 0.00000000001) + 1) * (1 / sqrt(rt_1)) << (dist_2 / (sqrt(pow(dist_2, 2)) + 0.00000000001) + 1) * (1 / sqrt(rt_2)) << (dist_3 / (sqrt(pow(dist_3, 2)) + 0.00000000001) + 1) * (1 / sqrt(rt_3)) << vx << vy << jx << jy << dv_1 * (dv_1 / (sqrt(pow(dv_1, 2)) + 0.000000000001) + 1) * (temp_1 / (sqrt(pow(temp_1, 2)) + 0.000000000001) + 1) << dv_2 * (dv_2 / (sqrt(pow(dv_2, 2)) + 0.000000000001) + 1) * (temp_2 / (sqrt(pow(temp_2, 2)) + 0.000000000001) + 1) << dv_3 * (dv_3 / (sqrt(pow(dv_3, 2)) + 0.000000000001) + 1) * (temp_3 / (sqrt(pow(temp_3, 2)) + 0.000000000001) + 1);
	hN << 0;

	// Provide defined weighting matrices:
	BMatrix W = eye<bool>( h.getDim() );
	BMatrix WN = eye<bool>( hN.getDim() );
	
	
	OCP ocp(0.0, 10.0, 100);

	ocp.subjectTo( f );

	ocp.subjectTo( -4.0 <= x <= 4.0 );
	ocp.subjectTo( -3.0 <= y <= 3.0 );	
	ocp.subjectTo( -1.0 <= vx <= 1.0 ); 
	ocp.subjectTo( -1.0 <= vy <= 1.0 ); 
	ocp.subjectTo( -2.0 <= ax <= 2.0 );
	ocp.subjectTo( -2.0 <= ay <= 2.0 );
	ocp.subjectTo( -10.0 <= jx <= 10.0 );
	ocp.subjectTo( -10.0 <= jy <= 10.0 );
	//ocp.subjectTo( -sqrt((x - xo_1) * (x - xo_1) + (y - yo_1) * (y - yo_1)) <= -0.35);
	//ocp.subjectTo( -sqrt((x - xo_2) * (x - xo_2) + (y - yo_2) * (y - yo_2)) <= -0.35);
	ocp.subjectTo( -sqrt((x - xm) * (x - xm) + (y - ym) * (y - ym)) <= -0.35);

	ocp.minimizeLSQ(W, h);
	ocp.minimizeLSQEndTerm(WN, hN);
	ocp.setNOD(11);
	

	OCPexport mpc( ocp );

	mpc.set( NUM_INTEGRATOR_STEPS,        30              );
	mpc.set( GENERATE_TEST_FILE,          NO              );
	mpc.set( GENERATE_MAKE_FILE,          YES             );	
	mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX,         YES);
	//mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);
	if (mpc.exportCode( "acadoOptim_alonso_TI_3obs" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}

