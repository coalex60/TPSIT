/*
 * lsm6dsl.c
 *
 *  Created on: Jan 31, 2023
 *      Author: Alessio
 */

#include "lsm6dsl.h"




int16_t lsm6dsReadId(i2c_transaction * i2cdata, int16_t * id)
{
	int16_t ret,reg=0x0F,val;//register whoami
	i2cdata->rx_buf = (void *) & ret;
	i2cdata->rx_len = 1;
	i2cdata->tx_buf = (void *) &reg;
	i2cdata->tx_len = 1;
	val = i2c_transfer(i2cdata);
	if (! val) (*id) = ret; else (*id) = 0;
return val;
}


int16_t lms6dsInit(i2c_transaction * i2cdata)
{

int8_t reg[10][2]={{0x10,0x60},{0x11,0x60},{0x12,0x44},{0x13,0x0},{0x14,0x0},{0x15,0x0},{0x16,0x0},
		{0x17,0x0},{0x18,0x0},{0x19,0x0}};
//CTRL1_XL ODR = 416Hz, Fullscale = +-2g,Bandwith =0,BWo = 400Hz
//CTRL2_G  ORD = 416Hz ,Fullscale = 250dps
//CTRL3_C = BDU, SIM,IF_INC
//CTRL4_C = 0  , CTRL5_C =0
//CTRL6_C = 0 ,  CTRL7_G = 0
//CTRL8_XL =0 , CTRL9_XL =0
//CTRL10_C = 0
osDelay(100);
int regint[2]={0x0D,0x03};
//INT1_CTRL = INT1_DRDY_G + INT1_DRDY_XL
int i,val;
for (i =0 ;i < 10; i++)
	{
	i2cdata->rx_len=0;
	i2cdata->rx_buf = NULL;
	i2cdata->tx_buf= (void *) reg[i];
	i2cdata->tx_len = 2;
	val = i2c_transfer(i2cdata);
	if (val) return 1;
	}
i2cdata->rx_len=0; //enable interrupt
i2cdata->rx_buf = NULL;
i2cdata->tx_buf= (void *) regint;
i2cdata->tx_len = 2;
val = i2c_transfer(i2cdata);
if (val) return 2;
return 0;
}
int16_t lms6dsReadData(i2c_transaction * i2cdata, lsm6dData * data)
{
	int8_t regstatus = 0x1E;//status register;
	int8_t ret_status;//value of status reg
	int8_t regini = 0x20;//registro iniziale
	uint8_t retval[14];
	int16_t val=0;
	do			//control of status register for data ready
		{
		i2cdata->rx_len= 1;//n.of byte read
		i2cdata->rx_buf = (void *) &ret_status;
		i2cdata->tx_buf= (void *) &regstatus;
		i2cdata->tx_len = 1;
		val = i2c_transfer(i2cdata);
		if (val) return 1;
		} while (ret_status != 7 ); //temperature,acceler.and giroscope data ready ?
//read register from 0x20 to 0x2D
	i2cdata->rx_len=14;//n.of byte read
	i2cdata->rx_buf = (void*) retval;//return value
	i2cdata->tx_buf= (void *) &regini;
	i2cdata->tx_len = 1;
	val = i2c_transfer(i2cdata);
	if (val) return 2;
	data->Temperature = (int16_t) ((uint16_t)retval[1]<<8)+((uint16_t)retval[0] );
	data->OUTX_G = (int16_t) ((uint16_t)retval[3] << 8)+((uint16_t) retval[2]);
	data->OUTY_G = ((uint16_t)retval[5] << 8)+((uint16_t) retval[4]);
	data->OUTZ_G = ((uint16_t)retval[7] << 8)+((uint16_t) retval[6]);
	data->OUTX_A = ((uint16_t)retval[9] << 8)+((uint16_t) retval[8]);
	data->OUTY_A = ((uint16_t)retval[11] << 8)+((uint16_t) retval[10]);
	data->OUTZ_A = ((uint16_t)retval[13] << 8)+((uint16_t) retval[12]);
	return 0;
}
