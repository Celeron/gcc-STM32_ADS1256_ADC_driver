#ifndef ADS1256_MACRO_H
#define ADS1256_MACRO_H


//-------------------------------------
// ADS1256 Command Definition (datasheet page 34)

// DATA READ
#define  ADS1256_COMMAND_RDATA      0x01  /* Read data once                        */
#define  ADS1256_COMMAND_RDATAC     0x03  /* Read data continously                 */
#define  ADS1256_COMMAND_SDATAC     0x0F  /* Stop reading data continously         */
// REGISTER READ
#define  ADS1256_COMMAND_RREG       0x10  /* Read From Register                    */
#define  ADS1256_COMMAND_WREG       0x50  /* Write To Register                     */
// CALIBRATION
#define  ADS1256_COMMAND_SELFCAL    0xF0  /* Offset and Gain Self-Calibration      */
#define  ADS1256_COMMAND_SELFOCAL   0xF1  /* Offset Self-Calibration               */
#define  ADS1256_COMMAND_SELFGCAL   0xF2  /* Gain Self-Calibration                 */
#define  ADS1256_COMMAND_SYSOCAL    0xF3  /* System Offset Calibration             */
#define  ADS1256_COMMAND_SYSGCAL    0xF4  /* System Gain Calibration               */
// SYSTEM CONTROL
#define  ADS1256_COMMAND_RESET      0xFE  /* Reset to Power-Up values              */
#define  ADS1256_COMMAND_SYNC       0xFC  /* Synchornize the A/D Conversion        */
#define  ADS1256_COMMAND_STANDBY    0xFD  /* Enter Sleep Mode                      */
#define  ADS1256_COMMAND_WAKEUP0    0x00  /* Completes SYNC and Exits Standby Mode */
#define  ADS1256_COMMAND_WAKEUP1    0xFF  /* Completes SYNC and Exits Standby Mode */



//-------------------------------------
// ADS1256 Register Map (datasheet page 30)

#define  ADS1256_REGISTER_STATUS  0x00  /* Status Control Register 0                      */
#define  ADS1256_REGISTER_MUX     0x01  /* Multiplexer Control Register 0                 */
#define  ADS1256_REGISTER_ADCON   0x02  /* A/D Control Register 0                         */
#define  ADS1256_REGISTER_DRATE   0x03  /* A/D Data Rate Control Register 0               */
#define  ADS1256_REGISTER_IO      0X04  /* GPIO Control Register 0                        */
#define  ADS1256_REGISTER_OFC0    0x05  /* Offset Calibration Coefficient Register 1      */
#define  ADS1256_REGISTER_OFC1    0x06  /* Offset Calibration Coefficient Register 2      */
#define  ADS1256_REGISTER_OFC2    0x07  /* Offset Calibration Coefficient Register 2      */
#define  ADS1256_REGISTER_FSC0    0x08  /* Full scale Callibration Coefficient Register 0 */
#define  ADS1256_REGISTER_FSC1    0x09  /* Full scale Callibration Coefficient Register 1 */
#define  ADS1256_REGISTER_FSC2    0x0A  /* Full scale Callibration Coefficient REgister 2 */



//---------------------------------------------------------------------------
// Формат регистра STATUS - Status Control Register 0 (see p30):
//
//     7     6     5     4   |     3     2     1     0
//    ID    ID    ID    ID   | ORDER  ACAL BUFEN  DRDY
//
//Reset Value = x1h

// ID3, ID2, ID1, ID0 = Factory Programmed Identification Bits (Read Only)
#define ADS1256_STATUS_ID          4
#define ADS1256_STATUS_IDMASK      0xF

// ORDER = Data Output Bit Order
//  Note: Input data is always shifted in most significant byte and bit first. 
//  Output data is always shifted out most significant byte first. 
//  The ORDER bit only controls the bit order of the output data within the byte.
#define ADS1256_STATUS_ORDER       3
#define ADS1256_STATUS_ORDER_MSB   0   /* Most significant Bit first (default) */
#define ADS1256_STATUS_ORDER_LSB   1   /* Least significant Bit first */

// ACAL = Auto Calibration
//  Note: When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command, 
//  that changes the PGA (bits 0-2 of ADCON register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the STATUS register) values.
#define ADS1256_STATUS_ACAL        2
#define ADS1256_STATUS_ACAL_OFF    0   /* Auto Calibration Disabled (default) */
#define ADS1256_STATUS_ACAL_ON     1   /* Auto Calibration Enabled */

// BUFEN = Analog Input Buffer Enable
#define ADS1256_STATUS_BUFEN       1
#define ADS1256_STATUS_BUFEN_OFF   0   /* Buffer Disabled (default) */
#define ADS1256_STATUS_BUFEN_ON    1   /* Buffer Enabled */

// DRDY = Data Ready (Read Only) = Duplicates the state of the DRDY pin
#define ADS1256_STATUS_DRDY        0
#define ADS1256_STATUS_DRDY_READY  0   /* DATA ready for read */
#define ADS1256_STATUS_DRDY_BUSY   1   /* Register updating pending... do not read! */



//---------------------------------------------------------------------------
// Формат регистра MUX - Input Multiplexer Control Register (see p31):
//
//     7      6      5      4  |     3      2      1      0
// PSEL3  PSEL2  PSEL1  PSEL0  | NSEL3  NSEL3  NSEL3  NSEL3
//
//Reset Value = 01h

// PSEL3, PSEL2, PSEL1, PSEL0 = Positive Input Channel (AINP) Select
// define multiplexer codes: 
#define ADS1256_MUXP_AIN0   0x00  /* (default)      */
#define ADS1256_MUXP_AIN1   0x10
#define ADS1256_MUXP_AIN2   0x20  /* (ADS1256 only) */
#define ADS1256_MUXP_AIN3   0x30  /* (ADS1256 only) */
#define ADS1256_MUXP_AIN4   0x40  /* (ADS1256 only) */
#define ADS1256_MUXP_AIN5   0x50  /* (ADS1256 only) */
#define ADS1256_MUXP_AIN6   0x60  /* (ADS1256 only) */
#define ADS1256_MUXP_AIN7   0x70  /* (ADS1256 only) */
#define ADS1256_MUXP_AINCOM 0x80  /* (when PSEL3=1, then PSEL2, PSEL1, PSEL0 are "don't care") */

// NSEL3, NSEL2, NSEL1, NSEL0 = Negative Input Channel (AINN) Select
// define multiplexer codes: 
#define ADS1256_MUXN_AIN0   0x00
#define ADS1256_MUXN_AIN1   0x01  /* (default)      */
#define ADS1256_MUXN_AIN2   0x02  /* (ADS1256 only) */
#define ADS1256_MUXN_AIN3   0x03  /* (ADS1256 only) */
#define ADS1256_MUXN_AIN4   0x04  /* (ADS1256 only) */
#define ADS1256_MUXN_AIN5   0x05  /* (ADS1256 only) */
#define ADS1256_MUXN_AIN6   0x06  /* (ADS1256 only) */
#define ADS1256_MUXN_AIN7   0x07  /* (ADS1256 only) */
#define ADS1256_MUXN_AINCOM 0x08  /* (when NSEL3=1, then NSEL2, NSEL1, NSEL0 are "don't care") */



//---------------------------------------------------------------------------
// Формат регистра ADCON - A/D Control Register (see p31):
//
//     7     6     5   |      4      3   |     2     1     0
//     0  CLK1  CLK0   |  SDCS1  SDCS0   |  PGA2  PGA1  PGA0
//
//Reset Value = 20h

// Bit7 Reserved = always 0 (Read Only)


// CLK1, CLK0 = D0/CLKOUT Clock Out Rate Setting
//  Note: When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.
#define ADS1256_ADCON_CLK       5
#define ADS1256_ADCON_CLKMASK   0x3
// define clock codes:
#define ADS1256_ADCON_CLK_OFF   0    /* Clock Out off                          */
#define ADS1256_ADCON_CLK_1     1    /* Clock Out Frequency = fCLKIN (default) */
#define ADS1256_ADCON_CLK_2     2    /* Clock Out Frequency = fCLKIN/2         */
#define ADS1256_ADCON_CLK_4     3    /* Clock Out Frequency = fCLKIN/4         */


// SDCS = Sensor Detection Current Sources
//  Note: The Sensor Detect Current Sources can be activated to verify the integrity of an external sensor supplying a signal to the ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.
#define ADS1256_ADCON_SDCS      3 
#define ADS1256_ADCON_SDCSMASK  0x3
// define current codes:
#define ADS1256_ADCON_SDCS_OFF  0    /* Sensor Detect Off (default) */
#define ADS1256_ADCON_SDCS_05   1    /* Sensor Detect Current 0.5uA */
#define ADS1256_ADCON_SDCS_2    2    /* Sensor Detect Current 2uA   */
#define ADS1256_ADCON_SDCS_10   3    /* Sensor Detect Current 10uA  */


// PGA = Programmable Gain Amplifier Setting
#define ADS1256_ADCON_PGA       0
#define ADS1256_ADCON_PGAMASK   0x3
// define gain codes:
#define ADS1256_ADCON_PGA_1     0x00   /* Vin = ±5V    (default) */
#define ADS1256_ADCON_PGA_2     0x01   /* Vin = ±2.5V            */
#define ADS1256_ADCON_PGA_4     0x02   /* Vin = ±1.25V           */
#define ADS1256_ADCON_PGA_8     0x03   /* Vin = ±0.625V          */
#define ADS1256_ADCON_PGA_16    0x04   /* Vin = ±312.5mV         */
#define ADS1256_ADCON_PGA_32    0x05   /* Vin = ±156.25mV        */
#define ADS1256_ADCON_PGA_64    0x06   /* Vin = ±78.125mV        */

// Для справки: входной динамический диапазон (VIN), при разных настройках коэффициента усиления (PGA)
//  PGA SETTING     VIN SCALE (VREF=2.5V)
//            1     ±5V    (без усиления, по умолчанию)
//            2     ±2.5V
//            4     ±1.25V
//            8     ±0.625V
//           16     ±312.5mV
//           32     ±156.25mV
//           64     ±78.125mV



//---------------------------------------------------------------------------
// Формат регистра DRATE - A/D Data Rate (see p32):
//
//     7    6    5    4    3    2    1    0
//   DR7  DR6  DR5  DR4  DR3  DR2  DR1  DR0
//
//Reset Value = F0h

//  NOTE: Data Rate vary depending on crystal frequency. Data rates listed below assumes the crystal frequency is 7.68Mhz 
//  Внутри микросхемы, по сути, реализован делитель тактовой частоты... Поэтому, при нестандартной F_CLKIN, Data rates scale linearly with F_CLKIN.

// DR = Data Rate Setting 
// define drate codes:
#define ADS1256_DRATE_30000SPS  0xF0  /* (default) */
#define ADS1256_DRATE_15000SPS  0xE0
#define ADS1256_DRATE_7500SPS   0xD0
#define ADS1256_DRATE_3750SPS   0xC0
#define ADS1256_DRATE_2000SPS   0xB0
#define ADS1256_DRATE_1000SPS   0xA1
#define ADS1256_DRATE_500SPS    0x92
#define ADS1256_DRATE_100SPS    0x82
#define ADS1256_DRATE_60SPS     0x72
#define ADS1256_DRATE_50SPS     0x63
#define ADS1256_DRATE_30SPS     0x53
#define ADS1256_DRATE_25SPS     0x43
#define ADS1256_DRATE_15SPS     0x33
#define ADS1256_DRATE_10SPS     0x23
#define ADS1256_DRATE_5SPS      0x13
#define ADS1256_DRATE_2_5SPS    0x03



//---------------------------------------------------------------------------
// Формат регистра I/O - GPIO Control Register (see p32):
//
//      7     6     5     4  |    3     2     1     0
//   DIR3  DIR2  DIR1  DIR0  | DIO3  DIO2  DIO1  DIO0
//
//Reset Value = E0h


// Note: The states of these bits control the operation of the general-purpose digital I/O pins. 
// The ADS1256 has 4 I/O pins: D3, D2, D1, and D0/CLKOUT. 
// The ADS1255 has two digital I/O pins: D1 and D0/CLKOUT. 
// When using an ADS1255, the register bits DIR3, DIR2, DIO3, and DIO2 can be read from and written to but have no effect.

// DIR3 = Digital I/O Direction for Pin D3 (used on ADS1256 only)
#define ADS1256_IO_DIR3      7
#define ADS1256_IO_DIR3_OUT  0  /* D3 is an output          */
#define ADS1256_IO_DIR3_IN   1  /* D3 is an input (default) */

// DIR2 = Digital I/O Direction for Pin D2 (used on ADS1256 only)
#define ADS1256_IO_DIR2      6
#define ADS1256_IO_DIR2_OUT  0  /* D2 is an output          */
#define ADS1256_IO_DIR2_IN   1  /* D2 is an input (default) */

// DIR1 = Digital I/O Direction for Pin D1
#define ADS1256_IO_DIR1      5
#define ADS1256_IO_DIR1_OUT  0  /* D1 is an output          */
#define ADS1256_IO_DIR1_IN   1  /* D1 is an input (default) */

// DIR0 = Digital I/O Direction for Pin D0
#define ADS1256_IO_DIR0      4
#define ADS1256_IO_DIR0_OUT  0  /* D0 is an output          */
#define ADS1256_IO_DIR0_IN   1  /* D0 is an input (default) */


// Note: Reading these bits will show the state of the corresponding digital I/O pin, whether if the pin is configured as an input or output by DIR3-DIR0. 
// When the digital I/O pin is configured as an output by the DIR bit, writing to the corresponding DIO bit will set the output state. 
// When the digital I/O pin is configured as an input by the DIR bit, writing to the corresponding DIO bit will have no effect. 
// When DO/CLKOUT is configured as an output and CLKOUT is enabled (using CLK1, CLK0 bits in the ADCON register), writing to DIO0 will have no effect.

// DIO3..DIO0 = Status of Digital I/O, Read Only
#define ADS1256_IO_DIO3      3
#define ADS1256_IO_DIO2      2
#define ADS1256_IO_DIO1      1
#define ADS1256_IO_DIO0      0



//---------------------------------------------------------------------------
// Формат регистра OFC0 - Offset Calibration Byte 0, least significant byte (see p33):
//
//      7      6      5      4      3      2      1      0
//  OFC07  OFC06  OFC05  OFC04  OFC03  OFC02  OFC01  OFC00
//
//Reset Value depends on calibration results...


// Формат регистра OFC1 - Offset Calibration Byte 1 (see p33):
//
//      7      6      5      4      3      2      1      0
//  OFC15  OFC14  OFC13  OFC12  OFC11  OFC10  OFC09  OFC08
//
//Reset Value depends on calibration results...


// Формат регистра OFC2 - Offset Calibration Byte 2, most significant byte (see p33):
//
//      7      6      5      4      3      2      1      0
//  OFC23  OFC22  OFC21  OFC20  OFC19  OFC18  OFC17  OFC16
//
//Reset Value depends on calibration results...



//---------------------------------------------------------------------------
// Формат регистра FSC0 - Full-scale Calibration Byte 0, least significant byte (see p33):
//
//      7      6      5      4      3      2      1      0
//  FSC07  FSC06  FSC05  FSC04  FSC03  FSC02  FSC01  FSC00
//
//Reset Value depends on calibration results...


// Формат регистра FSC1 - Full-scale Calibration Byte 1 (see p33):
//
//      7      6      5      4      3      2      1      0
//  FSC15  FSC14  FSC13  FSC12  FSC11  FSC10  FSC09  FSC08
//
//Reset Value depends on calibration results...


// Формат регистра FSC2 - Full-scale Calibration Byte 2, most significant byte (see p33):
//
//      7      6      5      4      3      2      1      0
//  FSC23  FSC22  FSC21  FSC20  FSC19  FSC18  FSC17  FSC16
//
//Reset Value depends on calibration results...


#endif  // ADS1256_MACRO_H
