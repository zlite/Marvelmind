#include <Firmata.h>

#define sqr(x) (x) * (x)

typedef struct
{
	long x;
	long y;
	long z;
} xyz;

class Marvelmind
{
	private:
		typedef struct
		{
			int adr;
			long x; // coordinates of beacon, mm
			long y;
			long z;
			bool ok;
		} tbeacon_pos;

		typedef union
		{
			byte b[2];
			unsigned int w;
			int wi;
		} uni_8x2_16;

		typedef union
		{
			byte b[4];
			float f;
			unsigned long v32;
			long vi32;
		} uni_8x4_32;

		static const int SPEED_SERIAL_DEFAULT = 9600;
		HardwareSerial *port_main;

		static const int HEDGEHOG_POS_PACKET_ID = 0x0001;
		static const int HEDGEHOG_CM_DATA_SIZE = 0x10;

		static const int HEDGEHOG_POS_HIGHRES_PACKET_ID = 0x0011;
		static const int HEDGEHOG_MM_DATA_SIZE = 0x16;

		static const int BEACONS_POS_PACKET_ID = 0x0002;
		static const int BEACONS_POS_HIGHRES_PACKET_ID = 0x0012;
		unsigned int hedgehog_data_id;

		long hedgehog_x, hedgehog_y; // coordinates of hedgehog (X,Y), mm
		long hedgehog_z; // height of hedgehog, mm

		bool hedgehog_pos_updated; // flag of new data from received
		bool high_resolution_mode;
		bool beacons_pos_appeared;

		static const int MAX_BEACONS_SAVE = 4;
		tbeacon_pos beacon_pos[MAX_BEACONS_SAVE]; // beacons positions

		static const int HEDGEHOG_BUF_SIZE = 128;
		byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
		byte hedgehog_serial_buf_ofs;
		byte hedgehog_packet_size;

		static const int MAX_SIZE_BUF_POS = 100;
		xyz buf_pos[ MAX_SIZE_BUF_POS ];
		int buf_pos_ofs;

		const int MAX_MOVE_IN_MILL;
	public:
		Marvelmind(
				int move_sec,
				int serial_n = 0,
				int serial_speed = SPEED_SERIAL_DEFAULT
		          );
		~Marvelmind();

		void process_date();

		xyz get_hedgehog_pos_xyz();
		long get_hedgehog_x();
		long get_hedgehog_y();
		long get_hedgehog_z();

		bool hedgehog_updated();
		bool beacons_pos_update();
		bool pos_mm();

		void hedgehog_set_crc16(byte *buf, byte size);
		HardwareSerial* (define_n_serial)(int serial_n = 0);

		void write_hedgehog(int serial_n = 0, int serial_speed = SPEED_SERIAL_DEFAULT);
		void write_beacon(int serial_n = 0, int serial_speed = SPEED_SERIAL_DEFAULT);
};
//==============================================================================