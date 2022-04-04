#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"
#include "main.h"
#include "drv_bmi088.h"

/******************************************************************************/
/*!                       Macro definitions                                   */
#define BMI088_SPI SPI1

#define BMI088_SPI_ACCEL 0
#define BMI088_SPI_GYRO 1

//ACCEL_CS = PA4, GYRO_CS = PB0(board c)
#define BMI088_ACCEL_NS_L HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
#define BMI088_ACCEL_NS_H HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
#define BMI088_GYRO_NS_L HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
#define BMI088_GYRO_NS_H HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

#define BMI08X_READ_WRITE_LEN  UINT8_C(64)

#define BMI08X_TIMEOUT_CNT 1680000

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection for accel */
uint8_t acc_dev_add;

/*! Variable that holds the I2C device address or SPI chip selection for gyro */
uint8_t gyro_dev_add;

/******************************************************************************/
/*!                User interface functions                                   */
//extern SPI_HandleTypeDef hspi1;

uint8_t spi_rw_byte(uint8_t byte){
	uint32_t timeout_cnt = 0;
	SET_BIT(BMI088_SPI->CR1, SPI_CR1_SPE);
    while((BMI088_SPI->SR & SPI_SR_TXE) == RESET){
		if(timeout_cnt < BMI08X_TIMEOUT_CNT){
			timeout_cnt++;
		}else return 0;
	}
	BMI088_SPI->DR = byte;
	timeout_cnt = 0;
    while((BMI088_SPI->SR & SPI_SR_RXNE) == RESET){
		if(timeout_cnt < BMI08X_TIMEOUT_CNT){
			timeout_cnt++;
		}else return 0;
	}
    return BMI088_SPI->DR;
//	uint8_t rx;
//	HAL_SPI_TransmitReceive(&hspi1,&byte,&rx,1,200);
//	return rx;
}


/*!
 * Delay function for stm32(systick)
 */
void bmi08x_delay_us(uint32_t period, void *intf_ptr)
{
    
    uint32_t tick_start,tick_now,reload;
    uint32_t tick_cnt = 0;
    reload = SysTick->LOAD;
    period = period * 168;
    tick_start = SysTick->VAL;
    while (1)
    {
        tick_now = SysTick->VAL;
        if (tick_now != tick_start)
        {
            if (tick_now < tick_start){
                tick_cnt += tick_start - tick_now;
            }else{
                tick_cnt += reload - tick_now + tick_start;
            }
            tick_start = tick_now;
            if (tick_cnt >= period){ break; }
        }
    }
}

/*!
 * SPI read function for stm32
 */
BMI08X_INTF_RET_TYPE bmi08x_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    if(dev_addr == BMI088_SPI_ACCEL){
        BMI088_ACCEL_NS_L;
        bmi08x_delay_us(2,intf_ptr);
        spi_rw_byte(reg_addr);
		//spi_rw_byte(reg_addr);
        while (len != 0){
            *reg_data = spi_rw_byte(0x55);
            reg_data++;len--;
        }
        BMI088_ACCEL_NS_H;
        bmi08x_delay_us(2,intf_ptr);
    }else{
        BMI088_GYRO_NS_L;
        bmi08x_delay_us(2,intf_ptr);
        spi_rw_byte(reg_addr);
		//spi_rw_byte(reg_addr);
        while (len != 0){
            *reg_data = spi_rw_byte(0x55);
            reg_data++;len--;
        }
        BMI088_GYRO_NS_H;
        bmi08x_delay_us(2,intf_ptr);
    }
    return 0;
}

/*!
 * SPI write function for stm32
 */
BMI08X_INTF_RET_TYPE bmi08x_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    if(dev_addr == BMI088_SPI_ACCEL){
        BMI088_ACCEL_NS_L;
        bmi08x_delay_us(2,intf_ptr);
        spi_rw_byte(reg_addr);
        while (len != 0){
            spi_rw_byte(*reg_data);
            reg_data++;len--;
        }
        BMI088_ACCEL_NS_H;
        //idle time between write access(typ. 2us)
        bmi08x_delay_us(10,intf_ptr);
    }else{
        BMI088_GYRO_NS_L;
        bmi08x_delay_us(2,intf_ptr);
        spi_rw_byte(reg_addr);
        while (len != 0){
            spi_rw_byte(*reg_data);
            reg_data++;len--;
        }
        BMI088_GYRO_NS_H;
        //idle time between write access(typ. 2us)
        bmi08x_delay_us(10,intf_ptr);
    }
    return 0;

}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *  
 */
int8_t bmi08x_interface_init(struct bmi08x_dev *bmi08x, uint8_t intf, enum bmi08x_variant variant)
{
    int8_t rslt = BMI08X_OK;

    if (bmi08x != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BMI08X_I2C_INTF)
        {
            //printf("I2C Interface \n");

            /* To initialize the user I2C function */
            // acc_dev_add = BMI08X_ACCEL_I2C_ADDR_PRIMARY;
            // gyro_dev_add = BMI08X_GYRO_I2C_ADDR_PRIMARY;
            // bmi08x->intf = BMI08X_I2C_INTF;
            // bmi08x->read = bmi08x_i2c_read;
            // bmi08x->write = bmi08x_i2c_write;
            return BMI08X_E_FEATURE_NOT_SUPPORTED;
        }
        /* Bus configuration : SPI */
        else if (intf == BMI08X_SPI_INTF)
        {
            //printf("SPI Interface \n");

            /* To initialize the user SPI function */
            bmi08x->intf = BMI08X_SPI_INTF;
            bmi08x->read = bmi08x_spi_read;
            bmi08x->write = bmi08x_spi_write;
            acc_dev_add = BMI088_SPI_ACCEL;
            gyro_dev_add = BMI088_SPI_GYRO;
        }

        /* Selection of bmi085 or bmi088 sensor variant */
        bmi08x->variant = variant;

        /* Assign accel device address to accel interface pointer */
        bmi08x->intf_ptr_accel = &acc_dev_add;

        /* Assign gyro device address to gyro interface pointer */
        bmi08x->intf_ptr_gyro = &gyro_dev_add;

        /* Configure delay in microseconds */
        bmi08x->delay_us = bmi08x_delay_us;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        bmi08x->read_write_len = BMI08X_READ_WRITE_LEN;
    }
    else
    {
        rslt = BMI08X_E_NULL_PTR;
    }

    return rslt;

}


/*!
 *  @brief Prints the execution status of the APIs.
 */
//void bmi08x_error_codes_print_result(const char api_name[], int8_t rslt)
//{
//    if (rslt != BMI08X_OK)
//    {
//        // printf("%s\t", api_name);
//        // if (rslt == BMI08X_E_NULL_PTR)
//        // {
//        //     printf("Error [%d] : Null pointer\r\n", rslt);
//        // }
//        // else if (rslt == BMI08X_E_COM_FAIL)
//        // {
//        //     printf("Error [%d] : Communication failure\r\n", rslt);
//        // }
//        // else if (rslt == BMI08X_E_DEV_NOT_FOUND)
//        // {
//        //     printf("Error [%d] : Device not found\r\n", rslt);
//        // }
//        // else if (rslt == BMI08X_E_OUT_OF_RANGE)
//        // {
//        //     printf("Error [%d] : Out of Range\r\n", rslt);
//        // }
//        // else if (rslt == BMI08X_E_INVALID_INPUT)
//        // {
//        //     printf("Error [%d] : Invalid input\r\n", rslt);
//        // }
//        // else if (rslt == BMI08X_E_CONFIG_STREAM_ERROR)
//        // {
//        //     printf("Error [%d] : Config stream error\r\n", rslt);
//        // }
//        // else if (rslt == BMI08X_E_RD_WR_LENGTH_INVALID)
//        // {
//        //     printf("Error [%d] : Invalid Read write length\r\n", rslt);
//        // }
//        // else if (rslt == BMI08X_E_INVALID_CONFIG)
//        // {
//        //     printf("Error [%d] : Invalid config\r\n", rslt);
//        // }
//        // else if (rslt == BMI08X_E_FEATURE_NOT_SUPPORTED)
//        // {
//        //     printf("Error [%d] : Feature not supported\r\n", rslt);
//        // }
//        // else if (rslt == BMI08X_W_FIFO_EMPTY)
//        // {
//        //     printf("Warning [%d] : FIFO empty\r\n", rslt);
//        // }
//        // else
//        // {
//        //     printf("Error [%d] : Unknown error code\r\n", rslt);
//        // }
//    }
//}

