/*
 * hts221.h
 *
 *  Created on: Oct 9, 2021
 *      Author: Alessio
 */

#ifndef FERMILIB_HTS221_H_
#define FERMILIB_HTS221_H_
#define hts221addr 0x5F;
typedef struct {
	uint8_t  H0_rH;
	uint8_t  H1_rH;
	uint16_t H0_T0_OUT;
	uint16_t H1_T0_OUT;
	uint16_t T0_degC;
	uint16_t T1_degC;
	uint16_t T0_OUT;
	uint16_t T1_OUT;
} calibVal; //dati calibrazione temperatura

int hts221_init(i2c_transaction * data);//inizializza
int hts221_ReadCal(i2c_transaction  * trans, calibVal * cdata );//legge parametri calib.
int hts221_ReadTempHum(i2c_transaction * trans,calibVal * cdata, float *Temp,float *Hum ); //legge temp e hum senza param.
int hts221_SensorId(i2c_transaction * data);//legge id sensore
int hts221_dataReady(i2c_transaction *data);


#endif /* FERMILIB_HTS221_H_ */
