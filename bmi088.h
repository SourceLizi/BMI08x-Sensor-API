/**
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file       bmi088.h
 * @date       27 May 2018
 * @version    1.1.0
 *
 */
/*! \file bmi088.h
 \brief Sensor Driver for BMI088 family of sensors */
#ifndef BMI088_H_
#define BMI088_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* header files */
#include "bmi08x_defs.h"
#if BMI08X_FEATURE_BMI088 == 1
/**********************************************************************************/
/* (extern) variable declarations */
/**********************************************************************************/
static const struct bmi08x_iir_filter_coef bmi08x_iir_coef_bw_47_odr_2000_hz =
{
	.filter_order = 4,
	.iir_a_coef = {1.0, -3.61432786,  4.91562219, -2.98049868, 0.67959907},
	.iir_b_coef = {0.00002467, 0.00009868, 0.00014802, 0.00009868, 0.00002467}
};

static const struct bmi08x_iir_filter_coef bmi08x_iir_coef_bw_47_odr_1000_hz =
{
	.filter_order = 4,
	.iir_a_coef = {1.0, -3.22963344,  3.97197803, -2.19773568, 0.4607062},
	.iir_b_coef = {0.00033219, 0.00132878, 0.00199317, 0.00132878, 0.00033219}
};

static const struct bmi08x_iir_filter_coef bmi08x_iir_coef_bw_47_odr_400_hz =
{
	.filter_order = 4,
	.iir_a_coef = {1.0, -2.08842538,  1.89653607, -0.81283712, 0.13768616},
	.iir_b_coef = {0.00830998, 0.03323993, 0.0498599 , 0.03323993, 0.00830998}
};

static const struct bmi08x_vre_filter_coef bmi08x_vre_coef_bw_116_odr_1600_hz =
{
		.filter_order = 2,
		.iir_a_coef = {1, -22444,  8609},
		.iir_b_coef = {2548, 5097, 2548},
		.iir_a_scale = {0, 14, 14},
		.iir_b_scale = {16, 16, 16}
};

static const struct bmi08x_vre_filter_coef bmi08x_vre_coef_std =
{
		.filter_order = 2,
		.iir_a_coef = {1, -28691, 12761},
		.iir_b_coef = {454, 908, 454},
		.iir_a_scale = {0, 14, 14},
		.iir_b_scale = {16, 16, 16}
};

/**********************************************************************************/
/* function prototype declarations */
/*!
 *  @brief This API is the entry point for bmi088 sensors.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel & gyro sensors.
 *
 *  @param[in,out] dev  : Structure instance of bmi08x_dev.
 *
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi088_init(struct bmi08x_dev *dev);

/*!
 *  @brief This API uploads the bmi088 config file onto the device.
 *
 *  @param[in,out] dev  : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi088_apply_config_file(struct bmi08x_dev *dev);

/*!
 *  @brief This API is used to enable/disable the data synchronization
 *  feature.
 *
 *  @param[in] sync_cfg : configure sync feature
 *  @param[in] dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi088_configure_data_synchronization(struct bmi08x_data_sync_cfg sync_cfg, struct bmi08x_dev *dev);

/*!
 *  @brief This API is used to enable/disable and configure the anymotion
 *  feature.
 *
 *  @param[in] anymotion_cfg : configure anymotion feature
 *  @param[in] dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi088_configure_anymotion(struct bmi08x_anymotion_cfg anymotion_cfg, struct bmi08x_dev *dev);

/*!
 *  @brief This API is used to enable/disable and configure the fast temperature update
 *  feature.
 *
 *  @param[in] fast_temp_cfg : configure fast temp feature
 *  @param[in] dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi088_configure_fast_temp(struct bmi08x_fast_temp_cfg fast_temp_cfg, struct bmi08x_dev *dev);

/*!
 *  @brief This API is used to enable/disable and configure the vre filter
 *  feature.
 *
 *  @param[in] vre_filter_cfg : configure vre filter feature
 *  @param[in] dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi088_configure_vre_filter(struct bmi088_vre_filter_cfg vre_filter_cfg, struct bmi08x_dev *dev);

/*!
 *  @brief This API reads the synchronized accel & gyro data from the sensor,
 *  store it in the bmi08x_sensor_data structure instance
 *  passed by the user.
 *
 *  @param[out] accel  : Structure pointer to store accel data
 *  @param[out] gyro   : Structure pointer to store gyro  data
 *  @param[in]  dev    : Structure instance of bmi08x_dev.
 *
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi088_get_synchronized_data(struct bmi08x_sensor_data *accel, struct bmi08x_sensor_data *gyro,
		const struct bmi08x_dev *dev);

/*!
 *  @brief This API reads the synchronized accel & gyro data from the sensor, applies the passed iir filter and
 *  store it in the bmi08x_sensor_data structure instance
 *  passed by the user.
 *
 *  @param[out] accel  : Structure pointer to store accel data
 *  @param[out] gyro   : Structure pointer to store gyro data
 *  @param[in/out] iir   : Structure pointer to read/store iir filter data
 *  @param[in]  dev    : Structure instance of bmi08x_dev.
 *
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi088_get_synchronized_data_filtered(struct bmi08x_sensor_data *accel, struct bmi08x_sensor_data *gyro, struct bmi08x_iir_filter *iir_acc, struct bmi08x_iir_filter *iir_gyr, const struct bmi08x_dev *dev);

/*!
 *  @brief This API configures the synchronization interrupt
 *  based on the user settings in the bmi08x_int_cfg
 *  structure instance.
 *
 *  @param[in] int_config : Structure instance of accel bmi08x_int_cfg.
 *  @param[in] dev         : Structure instance of bmi08x_dev.
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi088_set_data_sync_int_config(const struct bmi08x_int_cfg *int_config,	const struct bmi08x_dev *dev);

#endif
#ifdef __cplusplus
}
#endif

#endif /* BMI088_H_ */

/** @}*/
