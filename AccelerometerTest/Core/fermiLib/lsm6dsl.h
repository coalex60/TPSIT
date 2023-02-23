/*
 * lsm6dsl.h
 *
 *  Created on: Jan 31, 2023
 *      Author: Alessio
 */

#ifndef FERMILIB_LSM6DSL_H_
#define FERMILIB_LSM6DSL_H_

#include "cmsis_os.h"
#include "i2c.h"

typedef struct
{
	int16_t Temperature;//temperature
	int16_t OUTX_A; //accelerometer comp.X
	int16_t OUTY_A;//accelerometer comp.y
	int16_t OUTZ_A;//accelerometer comp.Z
	int16_t OUTX_G;//Giroscope comp.X
	int16_t OUTY_G;//Giroscope comp.y
	int16_t OUTZ_G;//Giroscope comp.z

} lsm6dData;



int16_t lsm6dsReadId(i2c_transaction * i2cdata, int16_t * id);
int16_t lms6dsInit(i2c_transaction * i2cdata);
int16_t lms6dsReadData(i2c_transaction * i2cdata, lsm6dData * data);

#endif /* FERMILIB_LSM6DSL_H_ */
