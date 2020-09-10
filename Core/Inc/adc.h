//
// Created by CM Geldenhuys on 2020/08/17.
//

#ifndef ADC_H_
#define ADC_H_

#include "main.h"

#ifndef ADC_HARDWARE_NSS
#ifndef ADC_nCS_Pin
#error "ADC chip select (CS) pin not defined [ADC_nCS_Pin]"
#endif // ADC_nCS_Pin

#ifndef ADC_nCS_GPIO_Port
#error "ADC chip select (CS) port not defined [ADC_nCS_GPIO_Port]"
#endif // ADC_nCS_GPIO_Port

#define ADC_CS_ENABLE() (HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_RESET))
#define ADC_CS_DISABLE() (HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_SET))
#define ADC_CS_STATE() (HAL_GPIO_ReadPin(ADC_nCS_GPIO_Port, ADC_nCS_Pin))
#define ADC_CS_IS_ENABLE() (ADC_CS_STATE() == GPIO_PIN_RESET)

#endif // ADC_HARDWARE_NSS

#ifndef ADC_nDRDY_Pin
#error "ADC data ready (DRDY) pin not defined [ADC_nDRDY_Pin]"
#endif // ADC_nDRDY_Pin

#ifndef ADC_nDRDY_GPIO_Port
#error "ADC data ready (DRDY) port not defined [ADC_nDRDY_GPIO_Port]"
#endif // ADC_nDRDY_GPIO_Port

#define ADC_IS_READY() (HAL_GPIO_ReadPin(ADC_nDRDY_GPIO_Port, ADC_nDRDY_Pin) == GPIO_PIN_SET)

#if !defined(ADC_BYTE_MODE_08) \
 || !defined(ADC_BYTE_MODE_16) \
 || !defined(ADC_BYTE_MODE_24) \
 || !defined(ADC_BYTE_MODE_32)
#define ADC_BYTE_MODE_16
#endif

#if defined(ADC_BYTE_MODE_08) \
 || defined(ADC_BYTE_MODE_24) \
 || defined(ADC_BYTE_MODE_32)
#warning "Byte mode not supported might experience undefined behaviour"
#endif

#define ADC_WAIT_DELTA_ (10)
#ifndef ADC_TIMEOUT
#define ADC_TIMEOUT HAL_MAX_DELAY
#endif // ADC_MAX_TIMEOUT


//                             3333222211110000
#define ADC_CMD_OP_NULL     (0b0000000000000000U)
#define ADC_CMD_OP_RESET    (0b0000000000010001U)
#define ADC_CMD_OP_STANDBY  (0b0000000000100010U)
#define ADC_CMD_OP_WAKEUP   (0b0000000000110011U)
#define ADC_CMD_OP_LOCK     (0b0000010101010101U)
#define ADC_CMD_OP_UNLOCK   (0b0000011001010101U)
#define ADC_CMD_OP_RREG     (0b1010000000000000U)
#define ADC_CMD_OP_WREG     (0b0110000000000000U)

#define ADC_ADR_ID            ADC_ADR(0x00U)
#define ADC_ADR_STATUS        ADC_ADR(0x01U)
#define ADC_ADR_MODE          ADC_ADR(0x02U)
#define ADC_ADR_CLOCK         ADC_ADR(0x03U)
#define ADC_ADR_GAIN          ADC_ADR(0x04U)
#define ADC_ADR_CFG           ADC_ADR(0x06U)
#define ADC_ADR_THRSHLD_MSB   ADC_ADR(0x07U)
#define ADC_ADR_IHRSHLD_LSB   ADC_ADR(0x08U)

#define ADC_ADR_CH0_CFG       ADC_ADR(0x09U)
#define ADC_ADR_CH0_OCAL_MSB  ADC_ADR(0x0AU)
#define ADC_ADR_CH0_OCAL_LSB  ADC_ADR(0x0BU)
#define ADC_ADR_CH0_GCAL_MSG  ADC_ADR(0x0CU)
#define ADC_ADR_CH0_GCAL_LSB  ADC_ADR(0x0DU)

#define ADC_ADR_CH1_CFG       ADC_ADR(0x0EU)
#define ADC_ADR_CH1_OCAL_MSB  ADC_ADR(0x0FU)
#define ADC_ADR_CH1_OCAL_LSB  ADC_ADR(0x10U)
#define ADC_ADR_CH1_GCAL_MSG  ADC_ADR(0x11U)
#define ADC_ADR_CH1_GCAL_LSB  ADC_ADR(0x12U)

#define ADC_ADR_CH2_CFG       ADC_ADR(0x13U)
#define ADC_ADR_CH2_OCAL_MSB  ADC_ADR(0x14U)
#define ADC_ADR_CH2_OCAL_LSB  ADC_ADR(0x15U)
#define ADC_ADR_CH2_GCAL_MSG  ADC_ADR(0x16U)
#define ADC_ADR_CH2_GCAL_LSB  ADC_ADR(0x17U)

#define ADC_ADR_CH3_CFG       ADC_ADR(0x18U)
#define ADC_ADR_CH3_OCAL_MSB  ADC_ADR(0x19U)
#define ADC_ADR_CH3_OCAL_LSB  ADC_ADR(0x1AU)
#define ADC_ADR_CH3_GCAL_MSG  ADC_ADR(0x1BU)
#define ADC_ADR_CH3_GCAL_LSB  ADC_ADR(0x1CU)

#define ADC_ADR_REGMAP_CRC    ADC_ADR(0x3EU)
#define ADC_ADR_RESERVED      ADC_ADR(0x3FU)

#define ADC_STATUS_LOCK                     (0x8000U)
#define ADC_STATUS_F_RESYNC                 (0x4000U)
#define ADC_STATUS_REG_MAP                  (0x2000U)
#define ADC_STATUS_CRC_ERR                  (0x1000U)
#define ADC_STATUS_CRC_TYPE                 (0x0800U)
#define ADC_STATUS_RESET                    (0x0400U)
#define ADC_STATUS_WLENGTH                  (0x0300U)
#define ADC_STATUS_WLENGTH_16               (0x0000U)
#define ADC_STATUS_WLENGTH_24               (0x0100U)
#define ADC_STATUS_WLENGTH_32_ZERO_PADDING  (0x0200U)
#define ADC_STATUS_WLENGTH_32_SIGN_EXTENDED (0x0300U)
#define ADC_STATUS_DRDY3                    (0x0008U)
#define ADC_STATUS_DRDY2                    (0x0004U)
#define ADC_STATUS_DRDY1                    (0x0002U)
#define ADC_STATUS_DRDY0                    (0x0001U)

#define ADC_MODE_REG_CRC_EN                 (0x2000U)
#define ADC_MODE_RX_CRC_EN                  (0x1000U)
#define ADC_MODE_CRC_TYPE                   (0x0800U)
#define ADC_MODE_CRC_TYPE_CCIT              (0x0800U)
#define ADC_MODE_CRC_TYPE_ANSI              (0x0800U)
#define ADC_MODE_RESET                      (0x0400U)
#define ADC_MODE_RESET_ACK                  (0x0000U)
#define ADC_MODE_WLENGTH                    (0x0300U)
#define ADC_MODE_WLENGTH_16                 (0x0300U)
#define ADC_MODE_WLENGTH_24                 (0x0300U)
#define ADC_MODE_WLENGTH_32_ZERO_PADDING    (0x0300U)
#define ADC_MODE_WLENGTH_32_SIGN_EXTENDED   (0x0300U)
#define ADC_MODE_TIMEOUT                    (0x0010U)
#define ADC_MODE_DRDY_SEL                   (0x000CU)
#define ADC_MODE_DRDY_HIZ                   (0x0002U)
#define ADC_MODE_DRDY_FMT                   (0x0001U)

#define ADC_CLOCK_CH3_EN      (0x0800U)
#define ADC_CLOCK_CH2_EN      (0x0400U)
#define ADC_CLOCK_CH1_EN      (0x0200U)
#define ADC_CLOCK_CH0_EN      (0x0100U)
#define ADC_CLOCK_OSR         (0x001CU)
#define ADC_CLOCK_OSR_128     (0x0000U)
#define ADC_CLOCK_OSR_256     (0x0004U)
#define ADC_CLOCK_OSR_512     (0x0008U)
#define ADC_CLOCK_OSR_1024    (0x000CU)
#define ADC_CLOCK_OSR_2048    (0x0008U)
#define ADC_CLOCK_OSR_4096    (0x0014U)
#define ADC_CLOCK_OSR_8192    (0x0018U)
#define ADC_CLOCK_OSR_16256   (0x001CU)
#define ADC_CLOCK_PWR         (0x0003U)
#define ADC_CLOCK_PWR_VLP     (0x0000U)
#define ADC_CLOCK_PWR_LP      (0x0001U)
#define ADC_CLOCK_PWR_HR      (0x0002U)

#define ADC_CMD_OP_ADR_POS  (7U)
#define ADC_ADR(adr)        ((adr) << ADC_CMD_OP_ADR_POS)

#define ADC_FRAME_NUM (6) // number of frames
#define ADC_SPS (4000)
#define ADC_FRAME_LEN (3) // Bytes

typedef enum {
    ADC_READY = 0x00000002U,
    ADC_FIRST_READ = 0x00000001U,
    ADC_IDLE = 0x00000000U,
    ADC_RESET
} ADC_state_e;

typedef struct {
    SPI_HandleTypeDef *spi;
    ADC_state_e state;
//    uint8_t           rx[ADC_FRAME_NUM][ADC_FRAME_SIZE];
} ADC_t;


int ADC_init (SPI_HandleTypeDef *interface);


void ADC_callbackDRDY ();

int ADC_sendCommand (uint16_t cmd);

#endif // ADC_H_
