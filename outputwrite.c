/*
 outputwrite — GPIO / Relay / ADC HAL helpers
 Version: 1.1.0  (updated 2026-06-22)
 Created by Aram Vartanyan, (C) 2024
 */

#include <stdio.h>
#include <esp_system.h>
#include <esp_log.h>
#include "outputwrite.h"

const static char *TAG = "InputOutput";

//https://docs.espressif.com/projects/esp-idf/en/v5.2.1/esp32/api-reference/peripherals/gpio.html
//Could be done differently: one init for inputs and another for outputs, so all functions are supported.

esp_err_t ioInit(gpio_num_t keyPinNumber, bool isOutput, bool isPullUp) {
    
    gpio_config_t io_conf;

    io_conf.pin_bit_mask = 1ULL << keyPinNumber;
    io_conf.intr_type = GPIO_INTR_DISABLE; //GPIO_INTR_DISABLE or GPIO_INTR_ANYEDGE
    
    if (isOutput) {
        io_conf.mode = GPIO_MODE_OUTPUT;
    } else {
        io_conf.mode = GPIO_MODE_INPUT;
    }
    
    if (isPullUp) {
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    } else {
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    }
    
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    
    return gpio_config(&io_conf);
}

esp_err_t ioInitPdown(gpio_num_t keyPinNumber, bool isOutput, bool isPullDwn) {
    
    gpio_config_t io_conf;

    io_conf.pin_bit_mask = 1ULL << keyPinNumber;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    
    if (isOutput) {
        io_conf.mode = GPIO_MODE_OUTPUT;
    } else {
        io_conf.mode = GPIO_MODE_INPUT;
    }
    
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    
    if (isPullDwn) {
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    } else {
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    }
    
    return gpio_config(&io_conf);
}

int ReadInput(gpio_num_t keyPinNumber) {
    return gpio_get_level(keyPinNumber);
}

void OutputWrite(bool on, gpio_num_t keyPinNumber, bool isReverse) {
    
    if (isReverse) {
        gpio_set_level(keyPinNumber, on ? 0 : 1);
    } else {
        gpio_set_level(keyPinNumber, on ? 1 : 0);
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)

//ADC Calibration

bool adcCalibrationInit(adc_unit_t Unit, adc_channel_t Channel, adc_atten_t Attenuation, adc_oneshot_unit_handle_t *adcHandle, adc_cali_handle_t *adcCalibrationHandle) {
    
    adc_cali_handle_t Handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool isCalibrated = false;
    
    //ADC1 Init
    adc_oneshot_unit_handle_t adcUnitHandle = NULL;
    adc_oneshot_unit_init_cfg_t initConfig = {
        .unit_id = Unit, //ADC_UNIT_1
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(adc_oneshot_new_unit(&initConfig, &adcUnitHandle));
    
    //ADC1 Config
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, //max supported width will be selected
        .atten = Attenuation,
    };
    
    ESP_ERROR_CHECK_WITHOUT_ABORT(adc_oneshot_config_channel(adcUnitHandle, Channel, &config));
    
    *adcHandle = adcUnitHandle;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!isCalibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = Unit,
            .chan = Channel,
            .atten = Attenuation,
            .bitwidth = ADC_BITWIDTH_DEFAULT, //max supported width will be selected
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &Handle);
        if (ret == ESP_OK) {
            isCalibrated = true;
        }
    }
#endif //ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!isCalibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = Unit,
            .atten = Attenuation,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &Handle);
        if (ret == ESP_OK) {
            isCalibrated = true;
        }
    }
#endif //ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED

    *adcCalibrationHandle = Handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !isCalibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return isCalibrated;
}

uint32_t adcRead(adc_oneshot_unit_handle_t adcHandle, adc_channel_t Channel, bool isCalibrated, adc_cali_handle_t adcCalibrationHandle) {
    int RawReading = 0;
    int Voltage = 0;

    ESP_ERROR_CHECK_WITHOUT_ABORT(adc_oneshot_read(adcHandle, Channel, &RawReading));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, Channel, RawReading);
    if (isCalibrated) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(adc_cali_raw_to_voltage(adcCalibrationHandle, RawReading, &Voltage));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, Channel, Voltage);
    }
    return Voltage;
}

static void adcCalibrationDeinit(adc_cali_handle_t Handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK_WITHOUT_ABORT(adc_cali_delete_scheme_curve_fitting(Handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK_WITHOUT_ABORT(adc_cali_delete_scheme_line_fitting(Handle));
#endif //ADC_CALI_SCHEME
}

//isCalibrated is the return of the adcCalibrationInit function
void adcDeinit(adc_oneshot_unit_handle_t adcHandle, bool isCalibrated, adc_cali_handle_t adcCalibrationHandle) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(adc_oneshot_del_unit(adcHandle));
    if (isCalibrated) {
        adcCalibrationDeinit(adcCalibrationHandle);
    }
}

#else //if the esp-idf version is below 5.0

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 1, 0)

#define DEFAULT_VREF    1100

//esp32 - ADC_BITWIDTH_9, ADC_BITWIDTH_10, ADC_BITWIDTH_11, ADC_BITWIDTH_12
//esp32s2 ADC_BITWIDTH_13
//esp32s3 ADC_BITWIDTH_12
//static const adc_channel_t Channel = ADC_CHANNEL_6;     //GPIO34 if ADC1

esp_adc_cal_characteristics_t adcCalibration(const adc_channel_t Channel, adc_bits_width_t WidthBit, adc_atten_t Attenuation) {
    
    static esp_adc_cal_characteristics_t *adc_chars;
    
    //Check if Two Point or Vref are burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Two Point: Supported");
    } else {
        ESP_LOGI(TAG, "eFuse Two Point: NOT supported");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Vref: Supported");
    } else {
        ESP_LOGI(TAG, "eFuse Vref: NOT supported");
    }

    //Configure ADC
    adc1_config_width(WidthBit);
    adc1_config_channel_atten(Channel, Attenuation); //ADC_ATTEN_DB_0 => 0 to 800 mV, ADC_ATTEN_DB_2_5 => 0 to 1100 mV

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, Attenuation, WidthBit, DEFAULT_VREF, adc_chars);
    
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "Characterized using Two Point Value");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "Characterized using eFuse Vref");
    } else {
        ESP_LOGI(TAG, "Characterized using Default Vref");
    }
    
    return adc_chars;
}

uint32_t adcRead(const adc_channel_t Channel, esp_adc_cal_characteristics_t *adc_chars) {
    uint8_t NoSamples = 5;
    uint32_t RawReading = 0;
    
    for (int i = 0; i < NoSamples; i++) {
        RawReading += adc1_get_raw((adc1_channel_t)Channel);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    RawReading /= NoSamples;
    return esp_adc_cal_raw_to_voltage(RawReading, adc_chars);
}

#endif

#endif //if the esp-idf version is below 5.0

