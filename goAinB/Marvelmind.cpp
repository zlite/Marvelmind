#include "Marvelmind.h"

Marvelmind::Marvelmind(
				int move_mill,
				int serial_n,
				int serial_speed
                      ):
				hedgehog_serial_buf_ofs(0),
				hedgehog_data_id(0),

				hedgehog_pos_updated(false),
				beacons_pos_appeared(false),
				high_resolution_mode(false),

				buf_pos_ofs(0),
				MAX_MOVE_IN_MILL(move_mill / 1000)
{

	for (int i = 0; i < MAX_BEACONS_SAVE; i++)
		beacon_pos[i].ok = false;

	port_main = define_n_serial(serial_n);
	port_main->begin(serial_speed);
};
//==============================================================================

Marvelmind::~Marvelmind()
{
	port_main->end();
};
//==============================================================================

xyz Marvelmind::get_hedgehog_pos_xyz()
{
	return buf_pos[ buf_pos_ofs ];
};
//==============================================================================

long Marvelmind::get_hedgehog_x()
{
	return hedgehog_x;
};
//==============================================================================
long Marvelmind::get_hedgehog_y()
{
	return hedgehog_y;
};
//==============================================================================
long Marvelmind::get_hedgehog_z()
{
	return hedgehog_z;
};
//==============================================================================

bool Marvelmind::beacons_pos_update()
{
	return beacons_pos_appeared;
};
//==============================================================================

bool Marvelmind::hedgehog_updated()
{
	return hedgehog_pos_updated;
};
//==============================================================================
bool Marvelmind::pos_mm()
{
	return high_resolution_mode;
};
//==============================================================================

void Marvelmind::process_date()
{
	hedgehog_pos_updated = false;
	int total_received_in_loop = 0;
	bool packet_received = false;
	int incoming_byte;
	bool good_byte;

	uni_8x2_16 un16;
	uni_8x4_32 un32;

	while (port_main->available() > 0)
	{
		if (hedgehog_serial_buf_ofs >= HEDGEHOG_BUF_SIZE)
		{
			hedgehog_serial_buf_ofs = 0; // restart bufer fill
			break; // buffer overflow
		}

		total_received_in_loop++;
		if (total_received_in_loop > 200)
			break; // too much data without required header

		incoming_byte = port_main->read();
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

	// if ( hedgehog_pos_updated && high_resolution_mode )
	// {
	// 	static long last_time = millis();
	// 	static long last_hedgehog_x;
	// 	static long last_hedgehog_y;
	// 	static long last_hedgehog_z;

	// 	long raz = millis() - last_time;
	// 	if ( abs( hedgehog_x - last_hedgehog_x ) / raz < MAX_MOVE_IN_MILL )
	// 		if ( abs( hedgehog_y - last_hedgehog_y ) / raz < MAX_MOVE_IN_MILL )
	// 			{
	// 				buf_pos[ buf_pos_ofs ].x = hedgehog_x;
	// 				buf_pos[ buf_pos_ofs ].y = hedgehog_y;
	// 				buf_pos[ buf_pos_ofs ].z = hedgehog_z;

	// 				if ( ++buf_pos_ofs >= MAX_SIZE_BUF_POS )
	// 				{
	// 					buf_pos_ofs = 0;

	// 					last_hedgehog_x = buf_pos[ MAX_SIZE_BUF_POS - 1 ].x;
	// 					last_hedgehog_x = buf_pos[ MAX_SIZE_BUF_POS - 1 ].y;
	// 					last_hedgehog_x = buf_pos[ MAX_SIZE_BUF_POS - 1 ].z;
	// 				}
	// 				last_time = millis();

	// 				last_hedgehog_x = buf_pos[ buf_pos_ofs - 1 ].x;
	// 				last_hedgehog_x = buf_pos[ buf_pos_ofs - 1 ].y;
	// 				last_hedgehog_x = buf_pos[ buf_pos_ofs - 1 ].z;
	// 			}
	// }
};
//==============================================================================

HardwareSerial* Marvelmind::define_n_serial(int serial_n)
{
	switch(serial_n)
	{
		case 0: return &Serial;
		case 1: return &Serial1;
		case 2: return &Serial2;
		case 3: return &Serial3;
	}
};
//==============================================================================

// Calculate CRC-16 of hedgehog packet
void Marvelmind::hedgehog_set_crc16(byte * buf, byte size)
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

void Marvelmind::write_hedgehog(int serial_n, int serial_speed)
{
	HardwareSerial *port_n = define_n_serial(serial_n);

	port_n->begin(serial_speed);
	port_n->print("hedgehog_x = ");
	port_n->println(hedgehog_x);

	port_n->print("hedgehog_y = ");
	port_n->println(hedgehog_y);

	port_n->print("hedgehog_z = ");
	port_n->println(hedgehog_z);

	port_n->println();

	port_n->end();
};
//==============================================================================

void Marvelmind::write_beacon(int serial_n, int serial_speed)
{
	HardwareSerial *port_n = define_n_serial(serial_n);

	port_n->begin(serial_speed);
	for (int i = 0; i < MAX_BEACONS_SAVE; i++)
	{
		port_n->print("beacon_pos[");
		port_n->print(i);
		port_n->println("]");

		port_n->print("adr = ");
		port_n->println(beacon_pos[i].adr);
		port_n->print("x = ");

		port_n->println(beacon_pos[i].x);
		port_n->print("y = ");

		port_n->println(beacon_pos[i].y);
		port_n->print("z = ");

		port_n->println(beacon_pos[i].z);
		port_n->print("ok = ");
		port_n->println(beacon_pos[i].ok);

		port_n->println();
	}
	port_n->end();
};
//==============================================================================