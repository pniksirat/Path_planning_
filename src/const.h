#ifndef CONST
#define CONST
int N_SAMPLES = 10;
static const double delta_ti = 0.02;
static const double dis_from_car_ahead=30;
static const double dis_from_car_back=15;

static const double spline_dis_points=30;
//vector<double>SIGMA_S= [10.0, 4.0, 2.0]; // s, s_dot, s_double_dot
//vector<double>SIGMA_D= [1.0, 1.0, 1.0];
static const vector<double> SIGMA={10.0,4.0,2.0,1.0,1.0,1.0};
double Vmax=49.2;
double SIGMA_T = 2.0;

double MAX_JERK = 10 ;// m/s/s/s
double MAX_ACCEL= 10 ;// m/s/s

double EXPECTED_JERK_IN_ONE_SEC = 2; // m/s/s
double EXPECTED_ACC_IN_ONE_SEC = 1 ;// m/s

double SPEED_LIMIT = 30;
float VEHICLE_RADIUS = 1.5; // model vehicle as circle to simplify collision detection

#endif