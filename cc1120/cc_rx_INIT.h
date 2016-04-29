// RX filter BW = 25.000000 
// Carrier frequency = 435.800049 
// PA ramping = true 
// Manchester enable = false 
// Packet length = 255 
// Device address = 0 
// TX power = 15 
// Symbol rate = 9.6 
// Whitening = true 
// Address config = No address check 
// Modulation format = 2-FSK 
// Deviation = 3.997803 
// Packet length mode = Variable 
// Bit rate = 9.6 
// Performance mode = High Performance 
// Packet bit length = 0 
//
// Rf settings for CC1120
//
#include cc_rx_INIT.h

void cc_rx_INIT() {

    halRfWriteReg(IOCFG3,0xB0);          //GPIO3 IO Pin Configuration
    halRfWriteReg(IOCFG2,0x06);          //GPIO2 IO Pin Configuration
    halRfWriteReg(IOCFG1,0x30);          //GPIO1 IO Pin Configuration
    halRfWriteReg(IOCFG0,0x40);          //GPIO0 IO Pin Configuration
    halRfWriteReg(SYNC3,0x93);           //Sync Word Configuration [31:24]
    halRfWriteReg(SYNC2,0x0B);           //Sync Word Configuration [23:16]
    halRfWriteReg(SYNC1,0x51);           //Sync Word Configuration [15:8]
    halRfWriteReg(SYNC0,0xDE);           //Sync Word Configuration [7:0]
    halRfWriteReg(SYNC_CFG1,0x0B);       //Sync Word Detection Configuration Reg. 1
    halRfWriteReg(SYNC_CFG0,0x17);       //Sync Word Length Configuration Reg. 0
    halRfWriteReg(DEVIATION_M,0x06);     //Frequency Deviation Configuration
    halRfWriteReg(MODCFG_DEV_E,0x03);    //Modulation Format and Frequency Deviation Configur..
    halRfWriteReg(DCFILT_CFG,0x4C);      //Digital DC Removal Configuration
    halRfWriteReg(PREAMBLE_CFG1,0x18);   //Preamble Length Configuration Reg. 1
    halRfWriteReg(PREAMBLE_CFG0,0x2A);   //Preamble Detection Configuration Reg. 0
    halRfWriteReg(FREQ_IF_CFG,0x40);     //RX Mixer Frequency Configuration
    halRfWriteReg(IQIC,0xC4);            //Digital Image Channel Compensation Configuration
    halRfWriteReg(CHAN_BW,0x08);         //Channel Filter Configuration
    halRfWriteReg(MDMCFG1,0x46);         //General Modem Parameter Configuration Reg. 1
    halRfWriteReg(MDMCFG0,0x05);         //General Modem Parameter Configuration Reg. 0
    halRfWriteReg(SYMBOL_RATE2,0x73);    //Symbol Rate Configuration Exponent and Mantissa [1..
    halRfWriteReg(SYMBOL_RATE1,0xA9);    //Symbol Rate Configuration Mantissa [15:8]
    halRfWriteReg(SYMBOL_RATE0,0x2A);    //Symbol Rate Configuration Mantissa [7:0]
    halRfWriteReg(AGC_REF,0x36);         //AGC Reference Level Configuration
    halRfWriteReg(AGC_CS_THR,0x19);      //Carrier Sense Threshold Configuration
    halRfWriteReg(AGC_GAIN_ADJUST,0x00); //RSSI Offset Configuration
    halRfWriteReg(AGC_CFG3,0x91);        //Automatic Gain Control Configuration Reg. 3
    halRfWriteReg(AGC_CFG2,0x20);        //Automatic Gain Control Configuration Reg. 2
    halRfWriteReg(AGC_CFG1,0xA9);        //Automatic Gain Control Configuration Reg. 1
    halRfWriteReg(AGC_CFG0,0xC3);        //Automatic Gain Control Configuration Reg. 0
    halRfWriteReg(FIFO_CFG,0x00);        //FIFO Configuration
    halRfWriteReg(DEV_ADDR,0x00);        //Device Address Configuration
    halRfWriteReg(SETTLING_CFG,0x0B);    //Frequency Synthesizer Calibration and Settling Con..
    halRfWriteReg(FS_CFG,0x14);          //Frequency Synthesizer Configuration
    halRfWriteReg(WOR_CFG1,0x08);        //eWOR Configuration Reg. 1
    halRfWriteReg(WOR_CFG0,0x21);        //eWOR Configuration Reg. 0
    halRfWriteReg(WOR_EVENT0_MSB,0x00);  //Event 0 Configuration MSB
    halRfWriteReg(WOR_EVENT0_LSB,0x00);  //Event 0 Configuration LSB
    halRfWriteReg(PKT_CFG2,0x04);        //Packet Configuration Reg. 2
    halRfWriteReg(PKT_CFG1,0x45);        //Packet Configuration Reg. 1
    halRfWriteReg(PKT_CFG0,0x20);        //Packet Configuration Reg. 0
    halRfWriteReg(RFEND_CFG1,0x0F);      //RFEND Configuration Reg. 1
    halRfWriteReg(RFEND_CFG0,0x00);      //RFEND Configuration Reg. 0
    halRfWriteReg(PA_CFG2,0x7F);         //Power Amplifier Configuration Reg. 2
    halRfWriteReg(PA_CFG1,0x56);         //Power Amplifier Configuration Reg. 1
    halRfWriteReg(PA_CFG0,0x7D);         //Power Amplifier Configuration Reg. 0
    halRfWriteReg(PKT_LEN,0xFF);         //Packet Length Configuration
    halRfWriteReg(IF_MIX_CFG,0x04);      //IF Mix Configuration
    halRfWriteReg(FREQOFF_CFG,0x22);     //Frequency Offset Correction Configuration
    halRfWriteReg(TOC_CFG,0x0B);         //Timing Offset Correction Configuration
    halRfWriteReg(MARC_SPARE,0x00);      //MARC Spare
    halRfWriteReg(ECG_CFG,0x00);         //External Clock Frequency Configuration
    halRfWriteReg(CFM_DATA_CFG,0x00);    //Custom frequency modulation enable
    halRfWriteReg(EXT_CTRL,0x01);        //External Control Configuration
    halRfWriteReg(RCCAL_FINE,0x00);      //RC Oscillator Calibration Fine
    halRfWriteReg(RCCAL_COARSE,0x00);    //RC Oscillator Calibration Coarse
    halRfWriteReg(RCCAL_OFFSET,0x00);    //RC Oscillator Calibration Clock Offset
    halRfWriteReg(FREQOFF1,0x00);        //Frequency Offset MSB
    halRfWriteReg(FREQOFF0,0x00);        //Frequency Offset LSB
    halRfWriteReg(FREQ2,0x6C);           //Frequency Configuration [23:16]
    halRfWriteReg(FREQ1,0xF3);           //Frequency Configuration [15:8]
    halRfWriteReg(FREQ0,0x34);           //Frequency Configuration [7:0]
    halRfWriteReg(IF_ADC2,0x02);         //Analog to Digital Converter Configuration Reg. 2
    halRfWriteReg(IF_ADC1,0xA6);         //Analog to Digital Converter Configuration Reg. 1
    halRfWriteReg(IF_ADC0,0x04);         //Analog to Digital Converter Configuration Reg. 0
    halRfWriteReg(FS_DIG1,0x00);         //Frequency Synthesizer Digital Reg. 1
    halRfWriteReg(FS_DIG0,0x5F);         //Frequency Synthesizer Digital Reg. 0
    halRfWriteReg(FS_CAL3,0x00);         //Frequency Synthesizer Calibration Reg. 3
    halRfWriteReg(FS_CAL2,0x20);         //Frequency Synthesizer Calibration Reg. 2
    halRfWriteReg(FS_CAL1,0x40);         //Frequency Synthesizer Calibration Reg. 1
    halRfWriteReg(FS_CAL0,0x0E);         //Frequency Synthesizer Calibration Reg. 0
    halRfWriteReg(FS_CHP,0x28);          //Frequency Synthesizer Charge Pump Configuration
    halRfWriteReg(FS_DIVTWO,0x03);       //Frequency Synthesizer Divide by 2
    halRfWriteReg(FS_DSM1,0x00);         //FS Digital Synthesizer Module Configuration Reg. 1
    halRfWriteReg(FS_DSM0,0x33);         //FS Digital Synthesizer Module Configuration Reg. 0
    halRfWriteReg(FS_DVC1,0xFF);         //Frequency Synthesizer Divider Chain Configuration ..
    halRfWriteReg(FS_DVC0,0x17);         //Frequency Synthesizer Divider Chain Configuration ..
    halRfWriteReg(FS_LBI,0x00);          //Frequency Synthesizer Local Bias Configuration
    halRfWriteReg(FS_PFD,0x50);          //Frequency Synthesizer Phase Frequency Detector Con..
    halRfWriteReg(FS_PRE,0x6E);          //Frequency Synthesizer Prescaler Configuration
    halRfWriteReg(FS_REG_DIV_CML,0x14);  //Frequency Synthesizer Divider Regulator Configurat..
    halRfWriteReg(FS_SPARE,0xAC);        //Frequency Synthesizer Spare
    halRfWriteReg(FS_VCO4,0x14);         //FS Voltage Controlled Oscillator Configuration Reg..
    halRfWriteReg(FS_VCO3,0x00);         //FS Voltage Controlled Oscillator Configuration Reg..
    halRfWriteReg(FS_VCO2,0x00);         //FS Voltage Controlled Oscillator Configuration Reg..
    halRfWriteReg(FS_VCO1,0x00);         //FS Voltage Controlled Oscillator Configuration Reg..
    halRfWriteReg(FS_VCO0,0xB4);         //FS Voltage Controlled Oscillator Configuration Reg..
    halRfWriteReg(GBIAS6,0x00);          //Global Bias Configuration Reg. 6
    halRfWriteReg(GBIAS5,0x02);          //Global Bias Configuration Reg. 5
    halRfWriteReg(GBIAS4,0x00);          //Global Bias Configuration Reg. 4
    halRfWriteReg(GBIAS3,0x00);          //Global Bias Configuration Reg. 3
    halRfWriteReg(GBIAS2,0x10);          //Global Bias Configuration Reg. 2
    halRfWriteReg(GBIAS1,0x00);          //Global Bias Configuration Reg. 1
    halRfWriteReg(GBIAS0,0x00);          //Global Bias Configuration Reg. 0
    halRfWriteReg(IFAMP,0x01);           //Intermediate Frequency Amplifier Configuration
    halRfWriteReg(LNA,0x01);             //Low Noise Amplifier Configuration
    halRfWriteReg(RXMIX,0x01);           //RX Mixer Configuration
    halRfWriteReg(XOSC5,0x0E);           //Crystal Oscillator Configuration Reg. 5
    halRfWriteReg(XOSC4,0xA0);           //Crystal Oscillator Configuration Reg. 4
    halRfWriteReg(XOSC3,0x03);           //Crystal Oscillator Configuration Reg. 3
    halRfWriteReg(XOSC2,0x04);           //Crystal Oscillator Configuration Reg. 2
    halRfWriteReg(XOSC1,0x03);           //Crystal Oscillator Configuration Reg. 1
    halRfWriteReg(XOSC0,0x00);           //Crystal Oscillator Configuration Reg. 0
    halRfWriteReg(ANALOG_SPARE,0x00);    //Analog Spare
    halRfWriteReg(PA_CFG3,0x00);         //Power Amplifier Configuration Reg. 3
    halRfWriteReg(WOR_TIME1,0x00);       //eWOR Timer Counter Value MSB
    halRfWriteReg(WOR_TIME0,0x00);       //eWOR Timer Counter Value LSB
    halRfWriteReg(WOR_CAPTURE1,0x00);    //eWOR Timer Capture Value MSB
    halRfWriteReg(WOR_CAPTURE0,0x00);    //eWOR Timer Capture Value LSB
    halRfWriteReg(BIST,0x00);            //MARC Built-In Self-Test
    halRfWriteReg(DCFILTOFFSET_I1,0x00); //DC Filter Offset I MSB
    halRfWriteReg(DCFILTOFFSET_I0,0x00); //DC Filter Offset I LSB
    halRfWriteReg(DCFILTOFFSET_Q1,0x00); //DC Filter Offset Q MSB
    halRfWriteReg(DCFILTOFFSET_Q0,0x00); //DC Filter Offset Q LSB
    halRfWriteReg(IQIE_I1,0x00);         //IQ Imbalance Value I MSB
    halRfWriteReg(IQIE_I0,0x00);         //IQ Imbalance Value I LSB
    halRfWriteReg(IQIE_Q1,0x00);         //IQ Imbalance Value Q MSB
    halRfWriteReg(IQIE_Q0,0x00);         //IQ Imbalance Value Q LSB
    halRfWriteReg(RSSI1,0x80);           //Received Signal Strength Indicator Reg. 1
    halRfWriteReg(RSSI0,0x00);           //Received Signal Strength Indicator Reg.0
    halRfWriteReg(MARCSTATE,0x41);       //MARC State
    halRfWriteReg(LQI_VAL,0x00);         //Link Quality Indicator Value
    halRfWriteReg(PQT_SYNC_ERR,0xFF);    //Preamble and Sync Word Error
    halRfWriteReg(DEM_STATUS,0x00);      //Demodulator Status
    halRfWriteReg(FREQOFF_EST1,0x00);    //Frequency Offset Estimate MSB
    halRfWriteReg(FREQOFF_EST0,0x00);    //Frequency Offset Estimate LSB
    halRfWriteReg(AGC_GAIN3,0x00);       //Automatic Gain Control Reg. 3
    halRfWriteReg(AGC_GAIN2,0xD1);       //Automatic Gain Control Reg. 2
    halRfWriteReg(AGC_GAIN1,0x00);       //Automatic Gain Control Reg. 1
    halRfWriteReg(AGC_GAIN0,0x3F);       //Automatic Gain Control Reg. 0
    halRfWriteReg(CFM_RX_DATA_OUT,0x00); //Custom Frequency Modulation RX Data
    halRfWriteReg(CFM_TX_DATA_IN,0x00);  //Custom Frequency Modulation TX Data
    halRfWriteReg(ASK_SOFT_RX_DATA,0x30);//ASK Soft Decision Output
    halRfWriteReg(RNDGEN,0x7F);          //Random Number Generator Value
    halRfWriteReg(MAGN2,0x00);           //Signal Magnitude after CORDIC [16]
    halRfWriteReg(MAGN1,0x00);           //Signal Magnitude after CORDIC [15:8]
    halRfWriteReg(MAGN0,0x00);           //Signal Magnitude after CORDIC [7:0]
    halRfWriteReg(ANG1,0x00);            //Signal Angular after CORDIC [9:8]
    halRfWriteReg(ANG0,0x00);            //Signal Angular after CORDIC [7:0]
    halRfWriteReg(CHFILT_I2,0x08);       //Channel Filter Data Real Part [18:16]
    halRfWriteReg(CHFILT_I1,0x00);       //Channel Filter Data Real Part [15:8]
    halRfWriteReg(CHFILT_I0,0x00);       //Channel Filter Data Real Part [7:0]
    halRfWriteReg(CHFILT_Q2,0x00);       //Channel Filter Data Imaginary Part [18:16]
    halRfWriteReg(CHFILT_Q1,0x00);       //Channel Filter Data Imaginary Part [15:8]
    halRfWriteReg(CHFILT_Q0,0x00);       //Channel Filter Data Imaginary Part [7:0]
    halRfWriteReg(GPIO_STATUS,0x00);     //General Purpose Input/Output Status
    halRfWriteReg(FSCAL_CTRL,0x01);      //Frequency Synthesizer Calibration Control
    halRfWriteReg(PHASE_ADJUST,0x00);    //Frequency Synthesizer Phase Adjust
    halRfWriteReg(PARTNUMBER,0x00);      //Part Number
    halRfWriteReg(PARTVERSION,0x00);     //Part Revision
    halRfWriteReg(SERIAL_STATUS,0x00);   //Serial Status
    halRfWriteReg(MODEM_STATUS1,0x01);   //Modem Status Reg. 1
    halRfWriteReg(MODEM_STATUS0,0x00);   //Modem Status Reg. 0
    halRfWriteReg(MARC_STATUS1,0x00);    //MARC Status Reg. 1
    halRfWriteReg(MARC_STATUS0,0x00);    //MARC Status Reg. 0
    halRfWriteReg(PA_IFAMP_TEST,0x00);   //Power Amplifier Intermediate Frequency Amplifier T..
    halRfWriteReg(FSRF_TEST,0x00);       //Frequency Synthesizer Test
    halRfWriteReg(PRE_TEST,0x00);        //Frequency Synthesizer Prescaler Test
    halRfWriteReg(PRE_OVR,0x00);         //Frequency Synthesizer Prescaler Override
    halRfWriteReg(ADC_TEST,0x00);        //Analog to Digital Converter Test
    halRfWriteReg(DVC_TEST,0x0B);        //Digital Divider Chain Test
    halRfWriteReg(ATEST,0x40);           //Analog Test
    halRfWriteReg(ATEST_LVDS,0x00);      //Analog Test LVDS
    halRfWriteReg(ATEST_MODE,0x00);      //Analog Test Mode
    halRfWriteReg(XOSC_TEST1,0x3C);      //Crystal Oscillator Test Reg. 1
    halRfWriteReg(XOSC_TEST0,0x00);      //Crystal Oscillator Test Reg. 0
    halRfWriteReg(RXFIRST,0x00);         //RX FIFO Pointer First Entry
    halRfWriteReg(TXFIRST,0x00);         //TX FIFO Pointer First Entry
    halRfWriteReg(RXLAST,0x00);          //RX FIFO Pointer Last Entry
    halRfWriteReg(TXLAST,0x00);          //TX FIFO Pointer Last Entry
    halRfWriteReg(NUM_TXBYTES,0x00);     //TX FIFO Status
    halRfWriteReg(NUM_RXBYTES,0x00);     //RX FIFO Status
    halRfWriteReg(FIFO_NUM_TXBYTES,0x0F);//TX FIFO Status
    halRfWriteReg(FIFO_NUM_RXBYTES,0x00);//RX FIFO Status
}