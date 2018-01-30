#include <Encoder.h>

#define sqr(x) (x) * (x)

enum direction{ back = 0, forward, stop, left, right };
class Car
{
	private:
		const int circle_r_car;
		const int circle_r_wheel;

		const int pin_left_engine_p;
		const int pin_left_engine_m;

		const int pin_right_engine_p;
		const int pin_right_engine_m;

		const int pin_encoder_a_1;  // номер вывода, подключенный к CLK енкодера
		const int pin_encoder_b_1;  // номер вывода контроллера, подключенный к DT енкодера

		const int pin_encoder_a_2;  // номер вывода, подключенный к CLK енкодера
		const int pin_encoder_b_2;  // номер вывода контроллера, подключенный к DT енкодера

		int count_turn_encode_1;
		int count_turn_encode_2;

		Encoder *encoder_left;
		Encoder *encoder_right;

		direction motion_direct_left_engine;
		direction motion_direct_right_engine;
		direction motion_direct_car;
	public:
		static const int MAX_SPEED = 255;

		Car(
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
		   );

		int get_value_encoder(int side);
		int get_status_engine(int side);
		int get_status_motion_car();

		void motion_turn_on_angle(double angle);
		void motion_turn(int direct);

		void motion_forward(int speed, int direct);
		void motion_stop();
		void motion_left_engine(int speed, int direct);
		void motion_right_engine(int speed, int direct);
};
//==============================================================================