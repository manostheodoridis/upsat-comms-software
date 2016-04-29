#ifndef __CC_COMMANDS_H
#define __CC_COMMANDS_H

#define RXFIRST                  0xD2
#define TXFIRST                  0xD3
#define RXLAST                   0xD4
#define TXLAST                   0xD5
#define NUM_TXBYTES              0xD6  /* Number of bytes in TXFIFO */
#define NUM_RXBYTES              0xD7  /* Number of bytes in RXFIFO */
#define FIFO_NUM_TXBYTES         0xD8
#define FIFO_NUM_RXBYTES         0xD9


/* DATA FIFO Access */
#define SINGLE_TXFIFO            0x3F      /*  TXFIFO  - Single accecss to Transmit FIFO */
#define BURST_TXFIFO             0x7F      /*  TXFIFO  - Burst accecss to Transmit FIFO  */
#define SINGLE_RXFIFO            0xBF      /*  RXFIFO  - Single accecss to Receive FIFO  */
#define BURST_RXFIFO             0xFF      /*  RXFIFO  - Busrrst ccecss to Receive FIFO  */

#define LQI_CRC_OK_BM            0x80
#define LQI_EST_BM               0x7F



/* Command strobe registers */
#define SRES                     0x30      /*  SRES    - Reset chip. */
#define SFSTXON                  0x31      /*  SFSTXON - Enable and calibrate frequency synthesizer. */
#define SXOFF                    0x32      /*  SXOFF   - Turn off crystal oscillator. */
#define SCAL                     0x33      /*  SCAL    - Calibrate frequency synthesizer and turn it off. */
#define SRX                      0x34      /*  SRX     - Enable RX. Perform calibration if enabled. */
#define STX                      0x35      /*  STX     - Enable TX. If in RX state, only enable TX if CCA passes. */
#define SIDLE                    0x36      /*  SIDLE   - Exit RX / TX, turn off frequency synthesizer. */
#define SWOR                     0x38      /*  SWOR    - Start automatic RX polling sequence (Wake-on-Radio) */
#define SPWD                     0x39      /*  SPWD    - Enter power down mode when CSn goes high. */
#define SFRX                     0x3A      /*  SFRX    - Flush the RX FIFO buffer. */
#define SFTX                     0x3B      /*  SFTX    - Flush the TX FIFO buffer. */
#define SWORRST                  0x3C      /*  SWORRST - Reset real time clock. */
#define SNOP                     0x3D      /*  SNOP    - No operation. Returns status byte. */
#define AFC                      0x37      /*  AFC     - Automatic Frequency Correction */

/* Chip states returned in status byte */
#define STATE_IDLE               0x00
#define STATE_RX                 0x10
#define STATE_TX                 0x20
#define STATE_FSTXON             0x30
#define STATE_CALIBRATE          0x40
#define STATE_SETTLING           0x50
#define STATE_RXFIFO_ERROR       0x60
#define STATE_TXFIFO_ERROR       0x70

#endif
