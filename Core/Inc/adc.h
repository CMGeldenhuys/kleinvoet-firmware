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

//                             3333222211110000
#define ADC_CMD_RES_RESET   (0b1111111100100100U) // 0xFF24

#define ADC_CMD_OP_ADR_POS  (7U)
#define ADC_ADR(adr)        ((adr) << ADC_CMD_OP_ADR_POS)

#define ADC_BYTE_0(op) ((op) & 0x00FFU)
#define ADC_BYTE_1(op) (ADC_BYTE_0 ((unsigned)(op) >> 8U))

#define ADC_FRAME_LENGTH  (6) // frames
#define ADC_SPS (4000)
#define ADC_BUF_LEN (ADC_FRAME_LENGTH * ADC_SPS) // Store 1 sec.

typedef enum {
    ADC_READY = 0x00000002U,
    ADC_FIRST_READ = 0x00000001U,
    ADC_IDLE = 0x00000000U,
    ADC_RESET
} ADC_state_e;

typedef struct {
    SPI_HandleTypeDef *spi;
    ADC_state_e       state;
//    uint16_t          buf[ADC_SPS][ADC_FRAME_LENGTH];
} ADC_t;


int ADC_init (SPI_HandleTypeDef *interface);

int32_t ADC_sendCommand (uint16_t cmd);

void ADC_callbackDRDY ();

#endif // ADC_H_
