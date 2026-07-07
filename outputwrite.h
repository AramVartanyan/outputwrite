/*
 outputwrite — GPIO / Relay / ADC HAL helpers
 Version: 1.1.0  (updated 2026-06-22)
 Created by Aram Vartanyan, (C) 2024
 */

#ifndef _OUTPUTWRITE_H_
#define _OUTPUTWRITE_H_

#define OUTPUTWRITE_VERSION "1.1.0"

//#pragma once
#include <driver/gpio.h>
#include "esp_idf_version.h"   // ESP_IDF_VERSION / ESP_IDF_VERSION_VAL used below

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Set a GPIO Pin for Accessory
 * @param keyPinNumber GPIO number.
 * @param isOutput set input/output false / true
 * @param isPullUp set if the pin is Pulled up false / true
 * @return
 * - ESP_OK success
 * - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ioInit(gpio_num_t keyPinNumber, bool isOutput, bool isPullUp);

/**
 * Set a GPIO Pin for Accessory
 * @param keyPinNumber GPIO number.
 * @param isOutput set input/output false / true
 * @param isPullDwn set if the pin is Pulled downt false / true
 * @return
 * - ESP_OK success
 * - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t ioInitPdown(gpio_num_t keyPinNumber, bool isOutput, bool isPullDwn);

/**
 * @brief  GPIO get input level
 * @param  gpio_num GPIO number.
 * @return
 *     - 0 the GPIO input level is 0
 *     - 1 the GPIO input level is 1
 */
int ReadInput(gpio_num_t keyPinNumber);

/** Drive the general purpose outputs
 *
 * @param on - output logical level 1 or 0 -> true or false
 * @param number of GPIO
 * @param isReverse true if the ouput logical levels are reversed
 */
void OutputWrite(bool on, gpio_num_t keyPinNumber, bool isReverse);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)

#include <soc/soc_caps.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

/**
 * @brief  Initialize and calibrate ESP ADC
 * @param  Unit ADC_UNIT_1 or ADC_UNIT_2 (if supported)
 * @param  Channel ADC_CHANNEL_0 to ADC_CHANNEL_9
 * @param  Attenuation Check chip documentation
 * ADC_ATTEN_DB_0   = 0,  ///<No input attenuation, ADC can measure up to approx.
 * ADC_ATTEN_DB_2_5 = 1,  ///<The input voltage of ADC will be attenuated extending the range of measurement by about 2.5 dB (1.33 x)
 * ADC_ATTEN_DB_6   = 2,  ///<The input voltage of ADC will be attenuated extending the range of measurement by about 6 dB (2 x)
 * ADC_ATTEN_DB_11  = 3,  ///<The input voltage of ADC will be attenuated extending the range of measurement by about 11 dB (3.55 x)
 * @param  adcHandle Output Handle pointer
 * @param  adcCalibrationHandle Output Calibration Handle pointer
 * @return
 *     - 0 is not calibrated
 *     - 1 calibration is succesful
 *
 * Note: On ESP32C3, ADC2 is no longer supported, due to its HW limitation.
 * Search for errata on espressif website for more details.
 *
 * Example:
 * adc_cali_handle_t OutHandle = NULL; //adcCalibrationHandle
 * bool isCalibrated = adcCalibrationInit(ADC_UNIT_1, ADC_CHANNEL_2, ADC_ATTEN_DB_11, &OutHandle);
 */
bool adcCalibrationInit(adc_unit_t Unit, adc_channel_t Channel, adc_atten_t Attenuation, adc_oneshot_unit_handle_t *adcHandle, adc_cali_handle_t *adcCalibrationHandle);

/**
 * @brief  Read the ADC in mV
 * @param  adcHandle Output Handle
 * @param  Channel ADC_CHANNEL_0 to ADC_CHANNEL_9
 * @param  isCalibrated bool value return of the adcCalibrationInit function
 * @param  adcCalibrationHandle Output Calibration Handle
 * @return Voltage value, mV
 */
uint32_t adcRead(adc_oneshot_unit_handle_t adcHandle, adc_channel_t Channel, bool isCalibrated, adc_cali_handle_t adcCalibrationHandle);

/**
 * @brief  Deinitialize the ADC parameters
 * @param  adcHandle Output Handle
 * @param  isCalibrated bool value return of the adcCalibrationInit function
 * @param  adcCalibrationHandle Output Calibration Handle
 */
void adcDeinit(adc_oneshot_unit_handle_t adcHandle, bool isCalibrated, adc_cali_handle_t adcCalibrationHandle);

#else //if the esp-idf version is below 5.0

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 1, 0)

#include <esp_adc_cal.h>

/**
 * @brief  Calibrate ESP ADC in esp-idf 4.x
 * @param  Channel ADC_CHANNEL_0 to ADC_CHANNEL_9
 * @param  WidthBit the ADC resolution - example: ADC_BITWIDTH_12
 * @param  Attenuation Check chip documentation
 */
esp_adc_cal_characteristics_t adcCalibration(const adc_channel_t Channel, adc_bits_width_t WidthBit, adc_atten_t Attenuation);

/**
 * @brief  Read the ADC in esp-idf 4.x
 * @param  Channel ADC_CHANNEL_0 to ADC_CHANNEL_9
 * @param  adc_chars ADC pointer returned by adcCalibration function
 * @return Voltage value, mV
 */
uint32_t adcRead(const adc_channel_t Channel, esp_adc_cal_characteristics_t *adc_chars);

#endif

#endif ////if the esp-idf version is below 5.0

#ifdef __cplusplus
}
#endif

#endif /* _OUTPUTWRITE_H_ */
