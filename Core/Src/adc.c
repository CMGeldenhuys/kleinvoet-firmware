//
// Created by devsploit on 2020/08/17.
//

#include "adc.h"
#include "logger.h"

int ADC_init(ADC_t *self, SPI_TypeDef interface)
{
}

int ADC_sendCommand_(ADC_t *self, uint8_t *cmd, size_t cmdLen, uint8_t *recBuf, size_t recLen);