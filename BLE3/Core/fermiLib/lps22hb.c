/*
 * lps22hb.c
 *
 *  Created on: Oct 16, 2021
 *      Author: Alessio
 */


#include "i2c.h"
#include "lps22hb.h"


uint8_t lps22hb_ReadId(i2c_transaction * data)
{

	uint8_t regStart = 0x0F;
	uint8_t retval;
	data->tx_buf = (void *)&regStart;
	data->tx_len = 1;
	data->rx_buf = (void *)&retval;
	data->rx_len = 1;
	i2c_transfer(data);
	return (retval );
}

uint8_t lps22hb_Init(i2c_transaction * data)
{
	uint8_t regValue[2],value = 0;
	regValue[0] = 0x10;//scrittura valore in REG_CTRL1
	//one shot
	//BDU = 0
	//SIM =1
	regValue[1]= 0x01;
	data->tx_buf =(void *)regValue;
	data->tx_len =2;
	data->rx_len =0;
	data->rx_buf = (void *)0;
	value = i2c_transfer(data);
	regValue[0] = 0x11; //scrittura REG_CTRL2
	regValue[1] = 0x10; //bit IF_ADD_INC alto (lettura di piu registri in serie)
	data->tx_buf = (void *)regValue;
	data->tx_len = 2;
	data->rx_buf = (void *) 0;
	data->rx_len = 0;
	value = i2c_transfer(data);
return value;
}

uint8_t lps22hb_ReadPT(i2c_transaction * data,PresTemp * pt)
{

	uint8_t regStart,regValue[5],ctrlr1[2],ret,stat;
	uint8_t PRESS_OUT_XL,PRESS_OUT_L,PRESS_OUT_H,Temp_OUT_H,Temp_OUT_L;
	uint32_t tmpress;
	uint16_t Temp_OUT;
	ctrlr1[0] = 0x11; //CTRL_2 register : start reading
	ctrlr1[1] = 0x11; //start reading one shot
	data->tx_buf = (void *)ctrlr1;
	data->tx_len  = 2;
	data->rx_buf = (void *) 0;
	data->rx_len  =0;
	ret = i2c_transfer(data);
	//read the STATUS reg
	do {
	ctrlr1[0] = 0x27;
	ctrlr1[1] = 0;
	data->tx_buf = (void *)& ctrlr1[0];
	data->tx_len  = 1;
	data->rx_buf = (void *) &ctrlr1[1];
	data->rx_len = 1;
	ret = i2c_transfer(data);
	stat = ctrlr1[1] & 3;
	} while (stat != 3); //repeat until data ready
	//read the pressure e temperature registers
	regStart = 0x28;
	data->tx_len = 1;
	data->tx_buf = (void *)&regStart;
	data->rx_len = 5;
	data->rx_buf = (void *)regValue;
	ret = i2c_transfer(data);
	PRESS_OUT_XL = regValue[0];
	PRESS_OUT_L = regValue[1];
	PRESS_OUT_H = regValue[2];
	Temp_OUT_L  = regValue[3];
	Temp_OUT_H = regValue[4];
	//calculate the pressure and temperature value
	tmpress = ((uint32_t) (PRESS_OUT_H << 16) )+((uint32_t)( PRESS_OUT_L << 8))+((uint32_t) (PRESS_OUT_XL));
	pt->Press = (uint32_t)(tmpress*100)/4096.0;
	Temp_OUT = ((uint16_t) (Temp_OUT_H << 8)) + (uint16_t)(Temp_OUT_L);
	pt->Temp = (int16_t )(Temp_OUT /10.0);
	return ret;
}
