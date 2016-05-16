/*# Modulation format = 2-FSK 
# Whitening = true 
# Bit rate = 1.2
# Device address = 0 
# Packet length = 3 
# Performance mode = High Performance 
# Packet bit length = 0 
# PA ramping = true 
# Carrier frequency = 435.800049 
# Deviation = 3.997803 
# Manchester enable = false 
# Address config = No address check 
# TX power = 6 dBm
# Packet length mode = Fixed 
# Symbol rate = 1.2
# RX filter BW = 25.000000 
 */
//
// Rf settings for CC1120
//
#include <cc_tx_init.h>

typedef struct
{
	unsigned int   addr;
	unsigned short dat;
}registerSetting_t;


static const registerSetting_t TX_preferredSettings[]=
{
		{IOCFG3,             0xB0},
		{IOCFG2,             0x06},
		{IOCFG1,             0xB0},
		{IOCFG0,             0x40},
		{SYNC3,              0x00},
		{SYNC2,              0x00},
		{SYNC1,              0x7A},
		{SYNC0,              0x0E},
		{SYNC_CFG1,          0x0B},
		{SYNC_CFG0,          0x0B},
		{DCFILT_CFG,         0x1C},
		{PREAMBLE_CFG1,      0x22},
		{IQIC,               0xC6},
		{CHAN_BW,            0x08},
		{MDMCFG0,            0x05},
		{SYMBOL_RATE2,       0x43},
		{SYMBOL_RATE1,       0xA9},
		{SYMBOL_RATE0,       0x2A},
		{AGC_REF,            0x20},
		{AGC_CS_THR,         0x19},
		{AGC_CFG1,           0xA9},
		{FIFO_CFG,           0x78},
		{SETTLING_CFG,       0x03},
		{FS_CFG,             0x14},
		//{PKT_CFG0,           0x28},
		{PKT_CFG0,           0x20},
		//{PA_CFG2,            0x2D},
		{PA_CFG2,            0x6B},  //4-PACFG2 0X26, 6dBm 0x2B 2dbm 0x22,8 DBM 2F
		{PA_CFG1,            0x56},
		{PA_CFG0,            0x7E},
		{PKT_LEN,            0xFF},
		{IF_MIX_CFG,         0x00},
		{FREQOFF_CFG,        0x22},
		{FREQ2,              0x6C},
		{FREQ1,              0xF3},
		{FREQ0,              0x34},
		{FS_DIG1,            0x00},
		{FS_DIG0,            0x5F},
		{FS_CAL1,            0x40},
		{FS_CAL0,            0x0E},
		{FS_DIVTWO,          0x03},
		{FS_DSM0,            0x33},
		{FS_DVC0,            0x17},
		{FS_PFD,             0x50},
		{FS_PRE,             0x6E},
		{FS_REG_DIV_CML,     0x14},
		{FS_SPARE,           0xAC},
		{FS_VCO4,            0x13},
		{FS_VCO1,            0xAC},
		{FS_VCO0,            0xB4},
		{XOSC5,              0x0E},
		{XOSC1,              0x03},
		{DCFILTOFFSET_I1,    0xF8},
		{DCFILTOFFSET_I0,    0x39},
		{DCFILTOFFSET_Q1,    0x0E},
		{DCFILTOFFSET_Q0,    0x9B},
		{IQIE_I1,            0xEF},
		{IQIE_I0,            0xDE},
		{IQIE_Q1,            0x02},
		{IQIE_Q0,            0x2F},
		{AGC_GAIN1,          0x13},
		{SERIAL_STATUS,      0x10},
};



void tx_registerConfig() {
	unsigned char writeByte;
	unsigned i;
	// Reset radio
	cc_tx_cmd(SRES);

	// Write registers to radio
	for(i = 0; i < (sizeof(TX_preferredSettings)/sizeof(registerSetting_t)); i++) {
		writeByte = TX_preferredSettings[i].dat;
		cc_tx_writeReg(TX_preferredSettings[i].addr, writeByte);
	}
}





#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX   1
#define FS_CHP_INDEX   2

void tx_manualCalibration() {

	uint8_t original_fs_cal2;
	uint8_t calResults_for_vcdac_start_high[3];
	uint8_t calResults_for_vcdac_start_mid[3];
	uint8_t marcstate;
	uint8_t writeByte;

	// 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
	writeByte = 0x00;
	cc_tx_writeReg(FS_VCO2, writeByte);

	// 2) Start with high VCDAC (original VCDAC_START + 2):
	cc_tx_readReg(FS_CAL2, &original_fs_cal2);
	writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
	cc_tx_writeReg(FS_CAL2, writeByte);

	// 3) Calibrate and wait for calibration to be done
	//   (radio back in IDLE state)
	cc_tx_cmd(SCAL);

	do {
		cc_tx_readReg(MARCSTATE, &marcstate);
	} while (marcstate != 0x41);

	// 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
	//    high VCDAC_START value
	cc_tx_readReg(FS_VCO2, &calResults_for_vcdac_start_high[FS_VCO2_INDEX]);
	cc_tx_readReg(FS_VCO4,&calResults_for_vcdac_start_high[FS_VCO4_INDEX]);
	cc_tx_readReg(FS_CHP, &calResults_for_vcdac_start_high[FS_CHP_INDEX]);

	// 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
	writeByte = 0x00;
	cc_tx_writeReg(FS_VCO2, writeByte);

	// 6) Continue with mid VCDAC (original VCDAC_START):
	writeByte = original_fs_cal2;
	cc_tx_writeReg(FS_CAL2, writeByte);

	// 7) Calibrate and wait for calibration to be done
	//   (radio back in IDLE state)
	cc_tx_cmd(SCAL);

	do {
		cc_tx_readReg(MARCSTATE, &marcstate);
	} while (marcstate != 0x41);

	// 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained
	//    with mid VCDAC_START value
	cc_tx_readReg(FS_VCO2, &calResults_for_vcdac_start_mid[FS_VCO2_INDEX]);
	cc_tx_readReg(FS_VCO4, &calResults_for_vcdac_start_mid[FS_VCO4_INDEX]);
	cc_tx_readReg(FS_CHP, &calResults_for_vcdac_start_mid[FS_CHP_INDEX]);

	// 9) Write back highest FS_VCO2 and corresponding FS_VCO
	//    and FS_CHP result
	if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
	calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
		writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
		cc_tx_writeReg(FS_VCO2, writeByte);
		writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
		cc_tx_writeReg(FS_VCO4, writeByte);
		writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
		cc_tx_writeReg(FS_CHP, writeByte);
	} else {
		writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
		cc_tx_writeReg(FS_VCO2, writeByte);
		writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
		cc_tx_writeReg(FS_VCO4, writeByte);
		writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
		cc_tx_writeReg(FS_CHP, writeByte);
	}
}



