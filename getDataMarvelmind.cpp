#include <stdlib.h>
#include <Arduino.h>

#define sqr(x) (x) * (x)

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

#define HEDGEHOG_POS_PACKET_ID 0x0001
#define HEDGEHOG_CM_DATA_SIZE 0x10

#define HEDGEHOG_POS_HIGHRES_PACKET_ID 0x0011
#define HEDGEHOG_MM_DATA_SIZE 0x16

#define BEACONS_POS_PACKET_ID 0x0002
#define BEACONS_POS_HIGHRES_PACKET_ID 0x0012
unsigned int hedgehog_data_id;

long hedgehog_x, hedgehog_y; // coordinates of hedgehog (X,Y), mm
long hedgehog_z; // height of hedgehog, mm

bool hedgehog_pos_updated; // flag of new data from received
bool high_resolution_mode;
bool beacons_pos_appeared;

#define MAX_BEACONS_SAVE 4
tbeacon_pos beacon_pos[MAX_BEACONS_SAVE]; // beacons positions

#define HEDGEHOG_BUF_SIZE 128
byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
byte hedgehog_serial_buf_ofs;
byte hedgehog_packet_size;

void hedgehog_set_crc16(byte *buf, byte size);
void setup_hedgehog();
void loop_hedgehog();

//==============================================================================

void setup_hedgehog()
{
	Serial.begin(9600); // hedgehog transmits data

	hedgehog_serial_buf_ofs = 0;
	hedgehog_data_id = 0;

	hedgehog_pos_updated = false;
	beacons_pos_appeared = false;

	for (int i = 0; i < MAX_BEACONS_SAVE; i++)
		beacon_pos[i].ok = false;
};
//==============================================================================

void loop_hedgehog()
{
	hedgehog_pos_updated = false;
	int total_received_in_loop = 0;
	bool packet_received = false;
	int incoming_byte;
	bool good_byte;

	uni_8x2_16 un16;
	uni_8x4_32 un32;

	while (Serial.available() > 0)
	{
		if (hedgehog_serial_buf_ofs >= HEDGEHOG_BUF_SIZE)
		{
			hedgehog_serial_buf_ofs = 0; // restart bufer fill
			break; // buffer overflow
		}

		total_received_in_loop++;
		if (total_received_in_loop > 200)
			break; // too much data without required header

		incoming_byte = Serial.read();
		good_byte = false;

		switch (hedgehog_serial_buf_ofs)
		{
			case 0:
			{
				incoming_byte = 0xff;
				good_byte = true;
				break;
			}
			case 1:
			{
				incoming_byte = 0x47;
				good_byte = true;
				break;
			}
			case 2:
			{
				good_byte = true;
				break;
			}
			case 3:
			{
				hedgehog_data_id = (( (unsigned int) incoming_byte ) << 8) + hedgehog_serial_buf[2];
				good_byte = (hedgehog_data_id == HEDGEHOG_POS_PACKET_ID) ||
				            (hedgehog_data_id == HEDGEHOG_POS_HIGHRES_PACKET_ID) ||
				            (hedgehog_data_id == BEACONS_POS_PACKET_ID) ||
				            (hedgehog_data_id == BEACONS_POS_HIGHRES_PACKET_ID);
				break;
			}
			case 4:
			{
				// save required packet size
				hedgehog_packet_size = incoming_byte + 7;
				switch (hedgehog_data_id)
				{
					case HEDGEHOG_POS_PACKET_ID:
					{
						good_byte = (incoming_byte == HEDGEHOG_CM_DATA_SIZE);
						break;
					}
					case HEDGEHOG_POS_HIGHRES_PACKET_ID:
					{
						good_byte = (incoming_byte == HEDGEHOG_MM_DATA_SIZE);
						break;
					}
					case BEACONS_POS_PACKET_ID:
					case BEACONS_POS_HIGHRES_PACKET_ID:
					{
						good_byte = true;
						break;
					}
				}
				break;
			}
			default:
			{
				good_byte = true;
				break;
			}
		}
		// end switch
		if (good_byte)
			;
		else
		{
			hedgehog_serial_buf_ofs = 0; // restart bufer fill
			hedgehog_data_id = 0;
			continue;
		}

		hedgehog_serial_buf[ hedgehog_serial_buf_ofs++ ] = incoming_byte;
		if (hedgehog_serial_buf_ofs > 5)
			if (hedgehog_serial_buf_ofs == hedgehog_packet_size)
			{
				// received packet with required header
				packet_received = true;
				hedgehog_serial_buf_ofs = 0; // restart bufer fill
				break;
			}
	}// end while

	if (packet_received)
	{
		hedgehog_set_crc16( &hedgehog_serial_buf[0], hedgehog_packet_size ); // calculate CRC checksum of packet
		if ( (hedgehog_serial_buf[ hedgehog_packet_size ] == 0) && (hedgehog_serial_buf[ hedgehog_packet_size + 1 ] == 0) )
		{
			// checksum success
			switch (hedgehog_data_id)
			{
				case HEDGEHOG_POS_PACKET_ID:
				{
					// coordinates of hedgehog (X,Y), cm ==> mm
					un16.b[0] = hedgehog_serial_buf[9];
					un16.b[1] = hedgehog_serial_buf[10];
					hedgehog_x = 10 * long(un16.wi);

					un16.b[0] = hedgehog_serial_buf[11];
					un16.b[1] = hedgehog_serial_buf[12];
					hedgehog_y = 10 * long(un16.wi);

					// height of hedgehog, cm==>mm
					un16.b[0] = hedgehog_serial_buf[13];
					un16.b[1] = hedgehog_serial_buf[14];
					hedgehog_z = 10 * long(un16.wi);

					hedgehog_pos_updated = true; // flag of new data from hedgehog received
					high_resolution_mode = false;
					break;
				}
				case HEDGEHOG_POS_HIGHRES_PACKET_ID:
				{
					// coordinates of hedgehog (X,Y), mm
					un32.b[0] = hedgehog_serial_buf[9];
					un32.b[1] = hedgehog_serial_buf[10];
					un32.b[2] = hedgehog_serial_buf[11];
					un32.b[3] = hedgehog_serial_buf[12];
					hedgehog_x = un32.vi32;

					un32.b[0] = hedgehog_serial_buf[13];
					un32.b[1] = hedgehog_serial_buf[14];
					un32.b[2] = hedgehog_serial_buf[15];
					un32.b[3] = hedgehog_serial_buf[16];
					hedgehog_y = un32.vi32;

					// height of hedgehog, mm
					un32.b[0] = hedgehog_serial_buf[17];
					un32.b[1] = hedgehog_serial_buf[18];
					un32.b[2] = hedgehog_serial_buf[19];
					un32.b[3] = hedgehog_serial_buf[20];
					hedgehog_z = un32.vi32;

					hedgehog_pos_updated = true; // flag of new data from hedgehog received
					high_resolution_mode = true;
					break;
				}
				case BEACONS_POS_PACKET_ID:
				{
					for (int i = 0; i < MAX_BEACONS_SAVE; i++)
						beacon_pos[i].ok = false;

					int n = hedgehog_serial_buf[5];
					if (n > MAX_BEACONS_SAVE)
						n = MAX_BEACONS_SAVE;

					for (int i = 0; i < n; i++)
					{
						int ofs = 6 + i * 8;

						beacon_pos[i].adr = hedgehog_serial_buf[ofs + 0];
						un16.b[0] = hedgehog_serial_buf[ofs + 1];
						un16.b[1] = hedgehog_serial_buf[ofs + 2];
						beacon_pos[i].x = 10 * long(un16.wi);

						un16.b[0] = hedgehog_serial_buf[ofs + 3];
						un16.b[1] = hedgehog_serial_buf[ofs + 4];
						beacon_pos[i].y = 10 * long(un16.wi);

						un16.b[0] = hedgehog_serial_buf[ofs + 5];
						un16.b[1] = hedgehog_serial_buf[ofs + 6];
						beacon_pos[i].z = 10 * long(un16.wi);

						beacon_pos[i].ok = true;
						beacons_pos_appeared = true;
					}
					break;
				}
				case BEACONS_POS_HIGHRES_PACKET_ID:
				{
					for (int i = 0; i < MAX_BEACONS_SAVE; i++)
						beacon_pos[i].ok = false;

					int n = hedgehog_serial_buf[5];
					if (n > MAX_BEACONS_SAVE)
						n = MAX_BEACONS_SAVE;

					for (int i = 0; i < n; i++)
					{
						int ofs = 6 + i * 14;

						beacon_pos[i].adr = hedgehog_serial_buf[ofs + 0];
						un32.b[0] = hedgehog_serial_buf[ofs + 1];
						un32.b[1] = hedgehog_serial_buf[ofs + 2];
						un32.b[2] = hedgehog_serial_buf[ofs + 3];
						un32.b[3] = hedgehog_serial_buf[ofs + 4];
						beacon_pos[i].x = un32.vi32;

						un32.b[0] = hedgehog_serial_buf[ofs + 5];
						un32.b[1] = hedgehog_serial_buf[ofs + 6];
						un32.b[2] = hedgehog_serial_buf[ofs + 7];
						un32.b[3] = hedgehog_serial_buf[ofs + 8];
						beacon_pos[i].y = un32.vi32;

						un32.b[0] = hedgehog_serial_buf[ofs + 9];
						un32.b[1] = hedgehog_serial_buf[ofs + 10];
						un32.b[2] = hedgehog_serial_buf[ofs + 11];
						un32.b[3] = hedgehog_serial_buf[ofs + 12];
						beacon_pos[i].z = un32.vi32;

						beacon_pos[i].ok = true;
						beacons_pos_appeared = true;
					}
					break;
				}
			}// switch(hedgehog_data_id)
		}// if CRC OK
	}// if (packet_received)
};
//==============================================================================

// Calculate CRC-16 of hedgehog packet
void hedgehog_set_crc16(byte * buf, byte size)
{
	uni_8x2_16 sum;
	byte shift_cnt;
	byte byte_cnt;
	sum.w = 0xffffU;
	for (byte_cnt = size; byte_cnt > 0; byte_cnt--)
	{
		sum.w = (unsigned int) ((sum.w/256U) * 256U + ((sum.w % 256U)^(buf[size - byte_cnt])));
		for (shift_cnt = 0; shift_cnt < 8; shift_cnt++)
		{
			if ((sum.w & 0x1) == 1)
				sum.w = (unsigned int) ((sum.w >> 1)^0xa001U);
			else
				sum.w >>= 1;
		}
	}
	buf[size] = sum.b[0];
	buf[size + 1] = sum.b[1]; // little endian
};
//==============================================================================