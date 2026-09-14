/*** FILE INCLUDE ****************************************************************
 *
 *    FILENAME:     var.h
 *    DATE:         21.01.2019
 *    AUTHOR:      (c) Nguyen Anh Danh
 *
 *    DESCRIPTION:
 *
 ********************************************************************************/
#ifndef __CRC_16_H
#define __CRC_16_H

#include 	"Arduino.h"
#include 	"stdbool.h" 
#include  <stddef.h>
#include  <stdint.h>

/*
 * #define CRC_POLY_xxxx
 *
 * The constants of the form CRC_POLY_xxxx define the polynomials for some well
 * known CRC calculations.
 */

#define		CRC_POLY_16		    		0xA001
#define		CRC_POLY_32		    		0xEDB88320ul
#define		CRC_POLY_64		    		0x42F0E1EBA9EA3693ull
#define		CRC_POLY_CCITT				0x1021
#define		CRC_POLY_DNP		  		0xA6BC
#define		CRC_POLY_KERMIT				0x8408
#define		CRC_POLY_SICK		  		0x8005

/*
 * #define CRC_START_xxxx
 *
 * The constants of the form CRC_START_xxxx define the values that are used for
 * initialization of a CRC value for common used calculation methods.
 */

#define		CRC_START_8		        0x00
#define		CRC_START_16		      0x0000
#define		CRC_START_MODBUS	    0xFFFF
#define		CRC_START_XMODEM	    0x0000
#define		CRC_START_CCITT_1D0F	0x1D0F
#define		CRC_START_CCITT_FFFF	0xFFFF
#define		CRC_START_KERMIT	    0x0000
#define		CRC_START_SICK		    0x0000
#define		CRC_START_DNP		      0x0000
#define		CRC_START_32		      0xFFFFFFFFul
#define		CRC_START_64_ECMA	    0x0000000000000000ull
#define		CRC_START_64_WE		    0xFFFFFFFFFFFFFFFFull

/*
 * Prototype list of global functions
 */

unsigned char *		checksum_NMEA(const unsigned char *input_str, unsigned char *result  );
uint8_t			crc_8(              const unsigned char *input_str, size_t num_bytes       );
//uint16_t		crc_16(             char *input_str, size_t num_bytes       );
uint16_t		crc_16(             char *input_str, size_t num_bytes       );

uint32_t		crc_32(             const unsigned char *input_str, size_t num_bytes       );
uint64_t		crc_64_ecma(        const unsigned char *input_str, size_t num_bytes       );
uint64_t		crc_64_we(          const unsigned char *input_str, size_t num_bytes       );
uint16_t		crc_ccitt_1d0f(     const unsigned char *input_str, size_t num_bytes       );
uint16_t		crc_ccitt_ffff(     const unsigned char *input_str, size_t num_bytes       );
uint16_t		crc_dnp(            const unsigned char *input_str, size_t num_bytes       );
uint16_t		crc_kermit(         const unsigned char *input_str, size_t num_bytes       );
uint16_t		crc_modbus(         const unsigned char *input_str, size_t num_bytes       );
uint16_t		crc_sick(           const unsigned char *input_str, size_t num_bytes       );
uint16_t		crc_xmodem(         const unsigned char *input_str, size_t num_bytes       );
uint8_t			update_crc_8(       uint8_t  crc, unsigned char c                          );
uint16_t		update_crc_16(      uint16_t crc, unsigned char c                          );
uint32_t		update_crc_32(      uint32_t crc, unsigned char c                          );
uint64_t		update_crc_64_ecma( uint64_t crc, unsigned char c                          );
uint16_t		update_crc_ccitt(   uint16_t crc, unsigned char c                          );
uint16_t		update_crc_dnp(     uint16_t crc, unsigned char c                          );
uint16_t		update_crc_kermit(  uint16_t crc, unsigned char c                          );
uint16_t		update_crc_sick(    uint16_t crc, unsigned char c, unsigned char prev_byte ); 

/*
 * Global CRC lookup tables
 */

extern const uint32_t	crc_tab32[];
extern const uint64_t	crc_tab64[];
static void  init_crc16_tab( void );

//  Test tinh crc16
//  char ctr[100]="";
//  sprintf(ctr, "011000120xxx001OKnguyenanhdanh");
//	uint16_t kq_CRC = 0;
//	int len = strlen(ctr);
//	kq_CRC  = crc_16( ctr, len);
//	
//	sprintf(ctr, "Kq Tinh ma CRC: %x", kq_CRC);
//  HAL_UART_Transmit(&huart2, (uint8_t *)ctr, strlen(ctr), 1000);  	
//  osDelay(1000); 
/*------------------------------------------------------------------------------*/
#endif
