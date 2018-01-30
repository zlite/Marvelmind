#include "Car.h"

Car::Car(
		int engine_l_p,
		int engine_l_m,
		int engine_r_p,
		int engine_r_m,

		int enc_a_1,
		int enc_b_1,
		int enc_a_2,
		int enc_b_2,

		int crc_r_car,
		int crc_r_wheel
        ):
		pin_left_engine_p(engine_l_p),
		pin_left_engine_m(engine_l_m),
		pin_right_engine_p(engine_r_p),
		pin_right_engine_m(engine_r_m),

		pin_encoder_a_1(enc_a_1),
		pin_encoder_b_1(enc_b_1),
		pin_encoder_a_2(enc_a_2),
		pin_encoder_b_2(enc_b_2),

		circle_r_car(crc_r_car),
		circle_r_wheel(crc_r_wheel),

		motion_direct_left_engine(stop),
		motion_direct_right_engine(stop),
		motion_direct_car(stop)
{
	encoder_left = new Encoder( pin_encoder_a_1, pin_encoder_b_1 );
	encoder_right = new Encoder( pin_encoder_a_2, pin_encoder_b_2 );
};
//==============================================================================

int Car::get_value_encoder(int side)
{
	if (side == right)
		return encoder_right->read() / 2;
	else return encoder_left->read() / 2;
};
int Car::get_status_engine(int side)
{
	if (side == right)
		return motion_direct_right_engine;
	else return motion_direct_left_engine;
};
//==============================================================================
int Car::get_status_motion_car()
{
	return motion_direct_car;
};
//==============================================================================

void Car::motion_turn_on_angle(double angle)
{
	//double len_turn_car =  2 * PI * circle_r_car / ( 2 * PI ) * angle;
	double len_turn_car = circle_r_car * angle;
	double angle_turn_wheel = len_turn_car / circle_r_wheel;

	//24 это кол-во щелчков на энкодере
	int count_turn = static_cast<int>( angle_turn_wheel / ( (2 * PI) / 24 ) );

	int cnt_turn_enc_l_old = encoder_left->read();
	int cnt_turn_enc_r_old = encoder_right->read();

	if (angle > 0)
		motion_turn(left);
	else motion_turn(right);

	while( ( encoder_left->read()/2 - cnt_turn_enc_l_old + encoder_right->read()/2 - cnt_turn_enc_r_old ) / 2 <= count_turn )
	{
		cnt_turn_enc_l_old = encoder_left->read();
		cnt_turn_enc_r_old = encoder_right->read();
	}

	motion_stop();
};
//==============================================================================

void Car::motion_turn(int side)
{
	if (side == left)
	{
		//left
		motion_left_engine( MAX_SPEED, back );
		motion_right_engine( MAX_SPEED, forward );
		motion_direct_car = left;
	}
	else
	{
		//right
		motion_left_engine( MAX_SPEED, forward );
		motion_right_engine( MAX_SPEED, back );
		motion_direct_car = right;
	}
};
//==============================================================================

void Car::motion_forward(int speed, int direc)
{
	if (direc == forward)
	{
		//forward
		motion_left_engine(speed, forward);
		motion_right_engine(speed, forward);
		motion_direct_car = forward;
	}
	else
	{
		//back
		motion_left_engine(speed, back);
		motion_right_engine(speed, back);
		motion_direct_car = back;
	}
};
//==============================================================================

void Car::motion_stop()
{
	if (motion_direct_left_engine != stop)
		motion_left_engine( 200, !motion_direct_left_engine );

	if (motion_direct_right_engine != stop)
		motion_right_engine( 200, !motion_direct_right_engine );

	if (motion_direct_left_engine != stop || motion_direct_right_engine != stop)
	{
		delay(20);
		motion_left_engine( 0, forward );
		motion_right_engine( 0, forward );
		motion_direct_left_engine = stop;
		motion_direct_right_engine = stop;
		motion_direct_car = stop;
	}
};
//==============================================================================

void Car::motion_right_engine(int speed, int direc)
{
	if (direc == forward)
	{
		//forward
		pinMode( pin_right_engine_m, OUTPUT );
		pinMode( pin_right_engine_p, INPUT );

		analogWrite( pin_right_engine_m, speed );
		motion_direct_right_engine = forward;
	}
	else
	{
		//back
		pinMode( pin_right_engine_m, INPUT );
		pinMode( pin_right_engine_p, OUTPUT );

		analogWrite( pin_right_engine_p, speed );
		motion_direct_right_engine = back;
	}
};
//==============================================================================

void Car::motion_left_engine(int speed, int direc)
{
	if (direc == forward)
	{
		//forward
		pinMode( pin_left_engine_m, OUTPUT );
		pinMode( pin_left_engine_p, INPUT );

		analogWrite( pin_left_engine_m, speed );
		motion_direct_left_engine = forward;
	}
	else
	{
		//back
		pinMode( pin_left_engine_m, INPUT );
		pinMode( pin_left_engine_p, OUTPUT );

		analogWrite( pin_left_engine_p, speed );
		motion_direct_left_engine = back;
	}
};
//==============================================================================