/*
 * hts221.c
 *
 *  Created on: Oct 9, 2021
 *      Author: Alessio
 */

#include "i2c.h"
#include "hts221.h"

int hts221_SensorId(i2c_transaction * data)
{
	uint8_t cmd,sensorId;
	cmd= 0x0F;//who am i register
	data->tx_buf = (void *)&cmd; //controlla che il valore resituito sia BC
	data->tx_len = 1;
	data->rx_buf = (void *)&sensorId;
	data->rx_len  =1;
	return (i2c_transfer(data));
	return sensorId;
}

int hts221_init(i2c_transaction * data)
{
uint8_t ret;
uint8_t ctrlr1[2];
ctrlr1[0]=0x20; //registro CTRL1_REG
ctrlr1[1] =0x84;
//ctrlr1[1] |=0x84; //porta a 1 i bit 7,2
//active mode bit7  =1, (bit2) BDU = 1, output register not updated until MSB and LSB reading
//one shot
data->tx_buf = (void*)ctrlr1;
data->tx_len = 2;
data->rx_buf =(void *)0;
data->rx_len = 0;
ret = i2c_transfer(data);
return ret;
}

int hts221_dataReady(i2c_transaction *data) //1 se dati pronti -0 se non pronti
{
uint8_t StartReg = 0x27; //Status register
uint8_t Rxval=0;
uint8_t retvalue =0;
data->tx_len = 1;
data->tx_buf = (void *)&StartReg;
data->rx_len = 1;
data->rx_buf = (void *)& Rxval;
retvalue  = (Rxval & 0x3);
return retvalue;
}


	int hts221_ReadCal (i2c_transaction  * trans, calibVal * cdata )
{

	uint8_t startReg,ret;
	uint8_t calibtmpHum1[4],calibtmpHum2[3],calibtmpHum3[6];//tre blocchi di calibrazione
	uint8_t T1_msb,T0_msb;
	uint16_t T0_degC_x8,T1_degC_x8;
	//lettura primo blocco calibrazione da 0x30 a 0x33
	startReg = 0x30;//registro iniziale calibrazione
	startReg |= 0x80; //autoincremento
	trans->tx_len = 1;
	trans->tx_buf = (void *)&startReg;   //legge 4 byte dei registri a partire da 0x30
	trans->rx_len = 4;
	trans->rx_buf = (void *)calibtmpHum1;
	ret = i2c_transfer(trans);
	cdata->H0_rH = (uint16_t) calibtmpHum1[0]>>1;
	cdata->H1_rH = (uint16_t)calibtmpHum1[1]>>1;
	T0_degC_x8 =(uint16_t) calibtmpHum1[2];
	T1_degC_x8 = (uint16_t) calibtmpHum1[3];
	//lettura secondo blocco calibrazione da 0x35 a 0x37
	startReg = 0x35;//lettura altri tre registri
	startReg |= 0x80;
	trans->tx_len = 1;
	trans->tx_buf = (void *)&startReg;
	trans->rx_buf = (void *)calibtmpHum2;
	trans->rx_len = 3;
	ret = i2c_transfer(trans);
	T0_msb = (calibtmpHum2[0] & 0x3);
	T1_msb = (calibtmpHum2[0] & 0xC) >> 2 ;
	cdata->T0_degC = (((uint16_t)T0_msb << 8) + ((uint16_t)T0_degC_x8)) >> 3;
	cdata->T1_degC = (((uint16_t)T1_msb << 8) + ((uint16_t)T1_degC_x8)) >> 3;
	cdata->H0_T0_OUT = ((uint16_t)calibtmpHum2[2]<< 8) + ((uint16_t) calibtmpHum2[1]);
	//lettura ultimo blocco calibrazione da 0x3A a 0x3F
	startReg = 0x3A; //lettura ultimi 6 valori
	startReg |= 0x80;
	trans->tx_len = 1;
	trans->tx_buf = (void *)&startReg;
	trans->rx_buf = (void *)calibtmpHum3;
	trans->rx_len = 6;
	ret = i2c_transfer(trans);
	cdata->H1_T0_OUT = ((uint16_t)calibtmpHum3[1]<< 8) + ((uint16_t) calibtmpHum3[0]);
	cdata->T0_OUT = ((uint16_t)calibtmpHum3[3]<< 8) + ((uint16_t) calibtmpHum3[2]);
	cdata->T1_OUT = ((uint16_t)calibtmpHum3[5]<< 8) + ((uint16_t) calibtmpHum3[4]);
return ret;
}

int hts221_ReadTempHum(i2c_transaction * trans, calibVal * cdata, int16_t *Temp,int16_t *Hum )
{
	uint8_t startReg;
	uint8_t rawtmpHum[4];
	//float tmpt,tmpH;
	int32_t tmpt,tmpH;
	uint16_t T_OUT,H_OUT;
	uint8_t ret;
	uint8_t ctrlr1[2];
	//ctrlr1[0] = 0x20;
	//ctrlr1[1] = 0x84 ; //register CTRL1_REG
	//trans->tx_buf = (void*)ctrlr1;
	//trans->tx_len = 2;
	//trans->rx_buf =(void *)0;
	//trans->rx_len = 0;
	//i2c_transfer(trans);
	ctrlr1[0] =  0x21;
	ctrlr1[1] = 0x01 ; //register CTRL2_REG (one shot)
	trans->tx_buf = (void*)ctrlr1;
	trans->tx_len = 2;
	trans->rx_buf =(void *)0;
	trans->rx_len = 0;
	i2c_transfer(trans);
	//control  status reg until data ready
	do {
	ctrlr1[0]=0x27;
	trans->tx_buf = (void *)& ctrlr1[0];
	trans->tx_len = 1;
	trans->rx_buf = (void *)& ctrlr1[1];
	trans->rx_len = 1;
	i2c_transfer(trans);
	ret = ctrlr1[1] & 3;
	} while (ret != 3);
	//lettura dati grezzi da 0x28 a 0x2B
	startReg = 0x28; //registro iniziale dati grezzi;
	startReg |= 0x80; //autoincremento registri
	trans->tx_len = 1;
	trans->tx_buf= (void *)&startReg;
	trans->rx_buf = (void *)rawtmpHum;
	trans->rx_len = 4;
	ret = i2c_transfer(trans);
    H_OUT  =((int16_t)rawtmpHum[1]<< 8) + ((int16_t) rawtmpHum[0]);
    T_OUT = ((int16_t)rawtmpHum[3]<< 8) + ((int16_t) rawtmpHum[2]);
    tmpt = (int32_t) (cdata->T1_degC - cdata->T0_degC)*(int32_t)((T_OUT-cdata->T0_OUT)*10);
    *Temp = (int32_t) (cdata->T0_degC *10  + (int32_t) tmpt/(int32_t)(cdata->T1_OUT - cdata->T0_OUT));
    tmpH = (int32_t) (cdata->H1_rH- cdata->H0_rH)*((int32_t)(H_OUT -cdata->H0_T0_OUT)*10);
    *Hum = (int32_t) ((cdata->H0_rH *10) + (int32_t)  tmpH /(int32_t)(cdata->H1_T0_OUT - cdata->H0_T0_OUT));
    if (*Hum > 1000) *Hum  =1000;
    return ret;
}
