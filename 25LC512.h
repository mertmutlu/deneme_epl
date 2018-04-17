#include "spi.h"

/** D E F I N E S **************************************************/
#define WREN       0x06// 0b00000110          // WREN command
#define WRDI       0x04// 0b00000100          // WRDI command
#define WRITE      0x02// 0b00000010          // WRITE command
#define READ       0x03// 0b00000011          // READ command
#define WRSR       0x01// 0b00000001          // WRSR command
#define RDSR       0x05// 0b00000101          // RDSR command
//#define CS          PORTFbits.RF2       // Chip select pin, PORTA pin 3
//#define SCK         PORTFbits.RF6       // Clock pin, PORTB pin 0
//#define SO          PORTFbits.RF8       // Serial output pin, PORTB pin 1
//#define SI          PORTFbits.RF7       // Serial input pin, PORTB pin 4

/** P R O T O T Y P E S ********************************************/
void LowDensByteWrite(unsigned char,unsigned int);                  // Low-density Byte Write function
void LowDensByteRead(unsigned char*,unsigned int);                   // Low-density Byte Read function
void LowDensPageWrite(unsigned char*,unsigned int,unsigned char);   // Low-density Page Write function
void LowDensSeqRead(unsigned char*,unsigned int,unsigned char);     // Low-density Sequential Read function
void HighDensByteWrite(unsigned char,unsigned short int);                 // High-density Byte Write function
//void HighDensByteRead(unsigned char*,unsigned short int);                  // High-density Byte Read function
void HighDensByteRead(unsigned char *data, unsigned short  address);
void HighDensPageWrite(unsigned char*,unsigned int,unsigned char);  // High-density Page Write function
void HighDensSeqRead(unsigned char*,unsigned int,unsigned char);    // High-density Sequential Read function
unsigned char ReadStatusReg(void);                                  // Read Status Register function
void WriteStatusReg(unsigned char);                                 // Write Status Register function
void WriteEnable(void);                                             // Write Enable function
void WriteDisable(void);                                            // Write Disable function
void WIP_Poll(void);                                                // WIP polling function
void write_u16_eeprom(unsigned short data,unsigned short address);
unsigned short read_u16_eeprom( unsigned short address);
unsigned int read_u32_eeprom( unsigned short address);
void write_u32_eeprom(unsigned int data,unsigned short address);
