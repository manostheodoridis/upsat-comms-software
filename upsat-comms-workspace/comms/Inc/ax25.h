#ifndef __AX25_H
#define __AX25_H

#include <stdint.h>

/* -*- c++ -*- */
/*
 * gr-satnogs: SatNOGS GNU Radio Out-Of-Tree Module
 *
 *  Copyright (C) 2016, Libre Space Foundation <http://librespacefoundation.org/>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "utils.h"
#include <limits.h>
#include <stddef.h>

#define AX25_MAX_ADDR_LEN 28
#define AX25_MAX_FRAME_LEN 256
#define AX25_MIN_ADDR_LEN 14
#define AX25_SYNC_FLAG 0x7E
#define AX25_MIN_CTRL_LEN 1
#define AX25_MAX_CTRL_LEN 2
#define AX25_CALLSIGN_MAX_LEN 6

#include "stm32f4xx_hal.h"
extern UART_HandleTypeDef huart5;
extern uint8_t uart_temp[100];
    
    extern const float AX25_SYNC_FLAG_MAP[8];
    extern const uint8_t AX25_SYNC_FLAG_MAP_BIN[8];
    //const size_t AX25_MIN_ADDR_LEN  = 14;
    //const size_t AX25_MAX_ADDR_LEN  = 28;
    //const size_t AX25_MIN_CTRL_LEN  = 1;
    //const size_t AX25_MAX_CTRL_LEN  = 2;
    //const size_t AX25_MAX_FRAME_LEN = 256;
    //const uint8_t AX25_SYNC_FLAG = 0x7E;
    //const uint8_t AX25_CALLSIGN_MAX_LEN = 6;
    //const float AX25_SYNC_FLAG_MAP[8] = {-1, 1, 1, 1, 1, 1, 1, -1};
    //const uint8_t AX25_SYNC_FLAG_MAP_BIN[8] = {0, 1, 1, 1, 1, 1, 1, 0};
    /**
     * AX.25 Frame types
     */
    typedef enum
    {
      AX25_I_FRAME, //!< AX25_I_FRAME Information frame
      AX25_S_FRAME, //!< AX25_S_FRAME Supervisory frame
      AX25_U_FRAME, //!< AX25_U_FRAME Unnumbered frame
      AX25_UI_FRAME /**!< AX25_UI_FRAME Unnumbered information frame */
    } ax25_frame_type_t;

    typedef enum
    {
      AX25_ENC_FAIL,
      AX25_ENC_OK
    } ax25_encode_status_t;

    typedef enum
    {
      AX25_DEC_FAIL,
      AX25_DEC_OK
    } ax25_decode_status_t;

    typedef struct
    {
      uint8_t address[AX25_MAX_ADDR_LEN];
      size_t address_len;
      uint16_t ctrl;
      size_t ctrl_len;
      uint8_t pid;
      uint8_t *info;
      size_t info_len;
      ax25_frame_type_t type;
    } ax25_frame_t;

    /**
     * Calculates the FCS of the AX25 frame
     * @param buffer data buffer
     * @param len size of the buffer
     * @return the FCS of the buffer
     */
    static inline uint16_t
    ax25_fcs(uint8_t *buffer, size_t len)
    {
      uint16_t fcs = 0xFFFF;
      while (len--) {
	fcs = (fcs >> 8) ^ crc16_ccitt_table_reverse[(fcs ^ *buffer++) & 0xFF];
      }
      return fcs ^ 0xFFFF;
    }
    
    /**
     * Creates the header field of the AX.25 frame
     * @param out the output buffer with enough memory to hold the address field
     * @param dest_addr the destination callsign address
     * @param dest_ssid the destination SSID
     * @param src_addr the callsign of the source
     * @param src_ssid the source SSID
     */
    static inline size_t
    ax25_create_addr_field (uint8_t *out, uint8_t * dest_addr,
			    uint8_t dest_ssid, uint8_t * src_addr,
			    uint8_t src_ssid)
    {
      uint16_t i = 0;
      
      for(i = 0; i < strnlen(dest_addr, AX25_MAX_ADDR_LEN); i++) {
	*out++ = dest_addr[i] << 1;
      }
      /*
       * Perhaps the destination callsign was smaller that the maximum allowed.
       * In this case the leftover bytes should be filled with space
       */
      for(; i < AX25_CALLSIGN_MAX_LEN; i++){
	*out++ = ' ' << 1;
      }
      /* Apply SSID, reserved and C bit */
      /* FIXME: C bit is set to 0 implicitly */
      *out++ = ((0x0F & dest_ssid) << 1) | 0x60;
      //*out++ = ((0b1111 & dest_ssid) << 1) | 0b01100000;

      for(i = 0; i < strnlen(src_addr, AX25_MAX_ADDR_LEN); i++) {
	*out++ = dest_addr[i] << 1;
      }
      for(; i < AX25_CALLSIGN_MAX_LEN; i++){
	*out++ = ' ' << 1;
      }
      /* Apply SSID, reserved and C bit. As this is the last address field
       * the trailing bit is set to 1.
       */
      /* FIXME: C bit is set to 0 implicitly */
      *out++ = ((0x0F & dest_ssid) << 1) | 0x61;
      //*out++ = ((0b1111 & dest_ssid) << 1) | 0b01100001;
      return (size_t)AX25_MIN_ADDR_LEN;
    }

    static inline size_t
    ax25_prepare_frame (uint8_t *out, const uint8_t *info, size_t info_len,
			ax25_frame_type_t type, uint8_t *addr, size_t addr_len,
			uint16_t ctrl, size_t ctrl_len)
    {
      uint16_t fcs;
      uint16_t i = 1;
      if(info_len > AX25_MAX_FRAME_LEN) {
	return 0;
      }

      out[0] = AX25_SYNC_FLAG;
      /* Insert address and control fields */
      if( addr_len == AX25_MIN_ADDR_LEN || addr_len == AX25_MAX_ADDR_LEN){
	memcpy(out + i, addr, addr_len);
	i += addr_len;
      }
      else{
	return 0;
      }

      if( ctrl_len == AX25_MIN_CTRL_LEN || ctrl_len == AX25_MAX_CTRL_LEN){
	memcpy(out + i, &ctrl, ctrl_len);
	i += ctrl_len;
      }
      else{
	return 0;
      }

      /*
       * Set the PID depending the frame type.
       * FIXME: For now, only the "No layer 3 is implemented" information is
       * inserted
       */
      if (type == AX25_I_FRAME || type == AX25_UI_FRAME) {
	out[i++] = 0xF0;
      }
      memcpy(out + i, info, info_len);
      i += info_len;

      /* Compute the FCS. Ignore the first flag byte */
      fcs = ax25_fcs(out + 1, i - 1);
      /* The MS bits are sent first ONLY at the FCS field */
      out[i++] = (fcs >> 8) & 0xFF;
      out[i++] = fcs & 0xFF;
      out[i++] = AX25_SYNC_FLAG;

      return i;
    }
  
       /**
     * Constructs an AX.25 by performing bit stuffing.
     * @param out the output buffer to hold the frame. To keep it simple,
     * each byte of the buffer holds only one bit. Also the size of the
     * buffer should be enough, such that the extra stuffed bits are fitting
     * on the allocated space.
     *
     * @param out_len due to bit stuffing the output size can vary. This
     * pointer will hold the resulting frame size after bit stuffing.
     *
     * @param buffer buffer holding the data that should be encoded.
     * Note that this buffer SHOULD contain the leading and trailing
     * synchronization flag, all necessary headers and the CRC.
     *
     * @param buffer_len the length of the input buffer.
     *
     * @return the resulting status of the encoding
     */
    static inline ax25_encode_status_t
    ax25_bit_stuffing (uint8_t *out, size_t *out_len, const uint8_t *buffer,
		       const size_t buffer_len)
    {
      uint8_t bit;
      uint8_t prev_bit = 0;
      size_t out_idx = 0;
      size_t bit_idx;
      size_t cont_1 = 0;
      size_t total_cont_1 = 0;
      size_t i;

      /* Leading FLAG field does not need bit stuffing */
      memcpy(out, AX25_SYNC_FLAG_MAP_BIN, 8 * sizeof(uint8_t));
      out_idx = 8;

      /* Skip the leading and trailing FLAG field */
      buffer++;
      for(i = 0; i < 8 * (buffer_len - 2); i++){
	bit = (buffer[i / 8] >> ( i % 8)) & 0x1;
	out[out_idx++] = bit;

	/* Check if bit stuffing should be applied */
	if(bit & prev_bit){
	  cont_1++;
	  total_cont_1++;
	  if(cont_1 == 4){
	    out[out_idx++] = 0;
	    cont_1 = 0;
	  }
	}
	else{
	  cont_1 = total_cont_1 = 0;
	}
	prev_bit = bit;

	/*
	 * If the total number of continuous 1's is 15 the the frame should be
	 * dropped
	 */
	if(total_cont_1 >= 14) {
	  return AX25_ENC_FAIL;
	}
      }

      /* Trailing FLAG field does not need bit stuffing */
      memcpy(out + out_idx, AX25_SYNC_FLAG_MAP_BIN, 8 * sizeof(uint8_t));
      out_idx += 8;

      *out_len = out_idx;
      return AX25_ENC_OK;
    }
    
    static inline ax25_encode_status_t ax25_nrz_bit_stuffing (float *out, size_t *out_len, const uint8_t *buffer,
			   const size_t buffer_len);

    //static inline ax25_encode_status_t ax25_bit_stuffing (uint8_t *out, size_t *out_len, const uint8_t *buffer,
//		       const size_t buffer_len);
    
    static inline ax25_decode_status_t
        ax25_decode (uint8_t *out, size_t *out_len,
    		 const uint8_t *ax25_frame, size_t len)
        {
          size_t i;
          size_t frame_start = UINT_MAX;
          size_t frame_stop = UINT_MAX;
          uint8_t res;
          size_t cont_1 = 0;
          size_t received_bytes = 0;
          size_t bit_cnt = 0;
          uint8_t decoded_byte = 0x0;
          uint16_t fcs;
          uint16_t recv_fcs;


          /* Start searching for the SYNC flag */
          for(i = 0; i < len - sizeof(AX25_SYNC_FLAG_MAP_BIN); i++) {
    	res = (AX25_SYNC_FLAG_MAP_BIN[0] ^ ax25_frame[i]) |
    	    (AX25_SYNC_FLAG_MAP_BIN[1] ^ ax25_frame[i + 1]) |
    	    (AX25_SYNC_FLAG_MAP_BIN[2] ^ ax25_frame[i + 2]) |
    	    (AX25_SYNC_FLAG_MAP_BIN[3] ^ ax25_frame[i + 3]) |
    	    (AX25_SYNC_FLAG_MAP_BIN[4] ^ ax25_frame[i + 4]) |
    	    (AX25_SYNC_FLAG_MAP_BIN[5] ^ ax25_frame[i + 5]) |
    	    (AX25_SYNC_FLAG_MAP_BIN[6] ^ ax25_frame[i + 6]) |
    	    (AX25_SYNC_FLAG_MAP_BIN[7] ^ ax25_frame[i + 7]);
    	/* Found it! */
    	if(res == 0){
    	  //std::cout << "Start found at " << i << std::endl;
              sprintf((char*)uart_temp, "AX Start found at %d\n", i);
              HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
    	  frame_start = i;
    	  break;
    	}
          }

          /* We failed to find the SYNC flag */
          if(frame_start == UINT_MAX){
    	//std::cout << "Frame start was not found" << std::endl;
            sprintf((char*)uart_temp, "AX Frame start was not found\n");
            HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
    	return AX25_DEC_FAIL;
          }

          for(i = frame_start + sizeof(AX25_SYNC_FLAG_MAP_BIN);
    	  i < len - sizeof(AX25_SYNC_FLAG_MAP_BIN) + 1; i++) {
    	/* Check if we reached the frame end */
    	res = (AX25_SYNC_FLAG_MAP_BIN[0] ^ ax25_frame[i]) |
    	    (AX25_SYNC_FLAG_MAP_BIN[1] ^ ax25_frame[i + 1]) |
    	    (AX25_SYNC_FLAG_MAP_BIN[2] ^ ax25_frame[i + 2]) |
    	    (AX25_SYNC_FLAG_MAP_BIN[3] ^ ax25_frame[i + 3]) |
    	    (AX25_SYNC_FLAG_MAP_BIN[4] ^ ax25_frame[i + 4]) |
    	    (AX25_SYNC_FLAG_MAP_BIN[5] ^ ax25_frame[i + 5]) |
    	    (AX25_SYNC_FLAG_MAP_BIN[6] ^ ax25_frame[i + 6]) |
    	    (AX25_SYNC_FLAG_MAP_BIN[7] ^ ax25_frame[i + 7]);
    	/* Found it! */
    	if(res == 0){
              sprintf((char*)uart_temp, "AX Stop found at %d\n", i);
              HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
    	  frame_stop = i;
    	  break;
    	}

    	if (ax25_frame[i]) {
    	  cont_1++;
    	  decoded_byte |= 1 << bit_cnt;
    	  bit_cnt++;
    	}
    	else {
    	  /* If 5 consecutive 1's drop the extra zero*/
    	  if (cont_1 >= 5) {
    	    cont_1 = 0;
    	  }
    	  else{
    	    bit_cnt++;
    	    cont_1 = 0;
    	  }
    	}

    	/* Fill the fully constructed byte */
    	if(bit_cnt == 8){
    	  out[received_bytes++] = decoded_byte;
    	  bit_cnt = 0;
    	  decoded_byte = 0x0;
    	}
          }

          if(frame_stop == UINT_MAX || received_bytes < AX25_MIN_ADDR_LEN){
            sprintf((char*)uart_temp, "AX Wrong frame size\n");
            HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
    	return AX25_DEC_FAIL;
          }

          /* Now check the CRC */
          fcs = ax25_fcs (out, received_bytes - sizeof(uint16_t));
          recv_fcs = (((uint16_t) out[received_bytes - 2]) << 8)
          	    | out[received_bytes - 1];

          if(fcs != recv_fcs) {
            sprintf((char*)uart_temp, "AX Wrong FCS\n");
            HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
    	return AX25_DEC_FAIL;
          }

          *out_len = received_bytes - sizeof(uint16_t);
          return AX25_DEC_OK;

        }

#endif
