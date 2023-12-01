/*
 * i2c.h
 *
 *  Created on: Jul 8, 2021
 *      Author: Alessio Cuoghi, Paolo Santinelli
 */

#ifndef FERMILIB_I2C_H_
#define FERMILIB_I2C_H_
#include "cmsis_os.h"
#include "stm32l4xx_hal.h"


typedef struct
{
    I2C_HandleTypeDef	*hi2c;			// I2c handle address
    uint16_t			 DevAddress; 	// I2C device address
    osMutexId_t 		 mutex;  		// I2c Mutex handle
    osSemaphoreId_t 	 semaphore; 	// I2c semaphore handle
	unsigned int 		 tx_len;		// Number of bytes to send to i2c slave
    void 				*tx_buf;		// tx data buffer
    unsigned int 		 rx_len;		// Number of bytes to read from i2c slave
    void 				*rx_buf;		// rx data buffer

} i2c_transaction;

int init_I2Cx(i2c_transaction *data);
int i2c_transfer(i2c_transaction *data);
void HAL_I2C_MasterRxCpltCallbackI2C2(I2C_HandleTypeDef * hi2c);
void HAL_I2C_MasterTxCpltCallbackI2C2(I2C_HandleTypeDef * hi2c);

void HAL_I2C_MasterRxCpltCallbackI2C1(I2C_HandleTypeDef * hi2c);
void HAL_I2C_MasterTxCpltCallbackI2C1(I2C_HandleTypeDef * hi2c);


#endif /* FERMILIB_I2C_H_ */
