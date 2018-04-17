#define  aed_offset_msg													0x04
#define  aed_offset_msg_mult										0x44// 68

#define  date_vp																0x0060
//--------------------------------------------------
#define 	header_1												0xAA
#define 	header_2												0xAA
//*****
#define 	test_header                     0x04
#define 	calibration_header              0x06
#define 	settings_header                 0x08
#define 	admin_header                    0x0A
//*****
#define  	pic_id_register									0x03
#define  	write_command										0x80
#define  	write_command_sram							0x82
void set_rtc(unsigned char year, unsigned char month,unsigned char date,unsigned char day , unsigned char hour,unsigned char min,
	unsigned char seconds);
void put_icon( unsigned short icon_number,unsigned short icon_vp );
void change_color( unsigned short sp, unsigned short color);
void send_header(void );
void goto_pic(unsigned short pic_id );
void input_number_unsigned16( unsigned short number,unsigned short number_vp );
void test_text(void);
void input_text( unsigned short index,unsigned short char_num, unsigned short vp);
void set_operating_timer(void);
void test_line(void);
void graph_init(void);
void lcd_date(void);
void graph(unsigned short x, unsigned short y, unsigned short vp );
void test_graph(void);
void aed_graph(unsigned short y);
void reset_graph(void);
void send_int(unsigned int  data);
void input_number_unsigned32( unsigned int number,unsigned short number_vp );
void put_key_popup( unsigned char key ) ;