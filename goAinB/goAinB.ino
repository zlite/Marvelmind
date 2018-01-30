#include "Marvelmind.h"
#include "Car.h"
typedef xyz vec;
const int count_check_point = 1;
xyz pos_check_point[count_check_point];
xyz last_pos;

// Marvelmind pos(30, 2);
Car car(2,3,4,5, 8,9,0,0, 0, 0);
HardwareSerial *port_main;
// ==============================================================================

double angle_for_turn(const vec & vec_static, const vec & vec_0);
double rad_to_deg(const double rad);
double deg_to_rad(const double deg);
HardwareSerial * define_n_serial(const int serial_n = 0);

int old_enc;
double angle = deg_to_rad( 270 );
int count_turn = static_cast<int>( angle / ( (2.0 * PI) / 30.0 ) );
// int count_turn = static_cast<int>( angle * 15.0 / PI );
// ==============================================================================
void setup()
{
	port_main = define_n_serial(1);
	port_main->begin(9600);

	// setup pos check point
	pos_check_point[0].x = 4540;
	pos_check_point[0].y = 280;

	// do
	// {
	// pos.process_date();
	// last_pos.x = pos.get_hedgehog_x();
	// last_pos.y = pos.get_hedgehog_y();
	// }
	// while ( !pos.hedgehog_updated() );
	old_enc = car.get_value_encoder( left );
	car.motion_forward( 40,forward );
	// port_main->println();
	// port_main->println();
	// port_main->println();
};
// ==============================================================================

void loop()
{
	// pos.process_date(); // Marvelmind hedgehog service loop
	// if (old_enc != car.get_value_encoder( left ))
	// {
	// 	port_main->print("encoder = ");
	// 	port_main->println( car.get_value_encoder( left ) );

	// 	old_enc = car.get_value_encoder(left);
	// 	port_main->print("count_turn = ");
	// 	port_main->println(count_turn);
	// }
	if ( car.get_value_encoder(left) >= count_turn )
	{
		car.motion_stop();
		port_main->print("encoder = ");
		port_main->println( car.get_value_encoder( left ) );

		old_enc = car.get_value_encoder(left);
		port_main->print("count_turn = ");
		port_main->println(count_turn);
	}

	// car.motion_forward(100, forward);
	// vec vec_motion_car = {
	// pos.get_hedgehog_x() - last_pos.x,
	// pos.get_hedgehog_y() - last_pos.y
	// };
	// vec vec_car_to_target = {
	// pos.get_hedgehog_x() - pos_check_point[0].x,
	// pos.get_hedgehog_y() - pos_check_point[0].y
	// };
	// last_pos.x = pos.get_hedgehog_x();
	// last_pos.y = pos.get_hedgehog_y();
};
// ==============================================================================

double angle_for_turn(const vec & vec_static, const vec & vec_0)
{
	double len_vec_static = sqrt(sqr(vec_static.x) + sqr(vec_static.y));
	double len_vec_0 = sqrt(sqr(vec_0.x) + sqr(vec_0.y));
	double scalar_vec_static_vec_0 = vec_static.x * vec_0.x + vec_static.y * vec_0.y;
	double angle_btn_Vec = acos(scalar_vec_static_vec_0 / (len_vec_static * len_vec_0));
	// double angle_axis_vec_static = acos( ( vec_static.x * 1 + vec_static.y * 0 ) / ( len_vec_static * 1 ) );
	double angle_axis_vec_static = acos(vec_static.x / len_vec_static);
	if (vec_static.y >= 0)
		;
	else
		angle_axis_vec_static = PI * 2 - angle_axis_vec_static;
	// double angle_axis_vec_0 = acos( ( vec_0.x * 1 + vec_0.y * 0 ) / ( len_vec_0 * 1 ) );
	double angle_axis_vec_0 = acos(vec_0.x / len_vec_0);
	if (vec_0.y >= 0)
		;
	else
		angle_axis_vec_0 = PI * 2 - angle_axis_vec_0;
	if (angle_axis_vec_0 - angle_axis_vec_static >= 0)
	{
		if (angle_axis_vec_0 - angle_axis_vec_static <= PI)
			return angle_btn_Vec;
		else
			return -angle_btn_Vec;
	}
	else
	{
		if (angle_axis_vec_static - angle_axis_vec_0 <= PI)
			return -angle_btn_Vec;
		else
			return angle_btn_Vec;
	}
};
// ==============================================================================

double rad_to_deg(const double rad)
{
	return rad / PI * 180;
};
// ==============================================================================

double deg_to_rad(const double deg)
{
	return deg * PI / 180;
};
// ==============================================================================

HardwareSerial * define_n_serial(const int serial_n)
{
	switch (serial_n)
	{
		case 0: return &Serial;
		case 1: return &Serial1;
		case 2: return &Serial2;
		case 3: return &Serial3;
	}
};

// ==============================================================================