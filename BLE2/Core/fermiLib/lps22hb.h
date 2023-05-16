/*
 * lps22hb.h
 *
 *  Created on: Oct 16, 2021
 *      Author: Alessio
 */

#ifndef FERMILIB_LPS22HB_H_
#define FERMILIB_LPS22HB_H_


typedef struct {
	uint32_t Press;
	int16_t Temp;
}PresTemp;



uint8_t lps22hb_ReadId(i2c_transaction * data);
uint8_t lps22hb_Init(i2c_transaction * data);
uint8_t lps22hb_ReadPT(i2c_transaction * data,PresTemp * pt);
#endif /* FERMILIB_LPS22HB_H_ */
