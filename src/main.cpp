/*
Copyright 2023 sb-child (sbchild0@gmail.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

#include <ch32v00x.h>
#include <debug.h>
#include <stdio.h>

// void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
extern "C" {
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
}
void Delay_Init(void);
void Delay_Ms(uint32_t n);
void Delay_Us(uint32_t n);

inline void _gpio_init(GPIO_TypeDef *x, uint16_t pin, GPIOMode_TypeDef mode,
                       GPIOSpeed_TypeDef speed) {
  GPIO_InitTypeDef GPIO_InitStructure = {0};
  GPIO_InitStructure.GPIO_Pin = pin;
  GPIO_InitStructure.GPIO_Mode = mode;
  GPIO_InitStructure.GPIO_Speed = speed;
  GPIO_Init(x, &GPIO_InitStructure);
}

#define GPIO_KBD_IN_1 GPIOC
#define GPIO_KBD_IN_2 GPIOC
#define GPIO_KBD_IN_3 GPIOC
#define GPIO_KBD_IN_4 GPIOC
#define GPIO_KBD_IN_5 GPIOC
#define GPIO_KBD_IN_6 GPIOC

GPIO_TypeDef *GPIO_KBD_IN[6] = {GPIO_KBD_IN_1, GPIO_KBD_IN_2, GPIO_KBD_IN_3,
                                GPIO_KBD_IN_4, GPIO_KBD_IN_5, GPIO_KBD_IN_6};

#define PIN_KBD_IN_1 GPIO_Pin_1
#define PIN_KBD_IN_2 GPIO_Pin_2
#define PIN_KBD_IN_3 GPIO_Pin_3
#define PIN_KBD_IN_4 GPIO_Pin_4
#define PIN_KBD_IN_5 GPIO_Pin_5
#define PIN_KBD_IN_6 GPIO_Pin_6

uint16_t PIN_KBD_IN[6] = {PIN_KBD_IN_1, PIN_KBD_IN_2, PIN_KBD_IN_3,
                          PIN_KBD_IN_4, PIN_KBD_IN_5, PIN_KBD_IN_6};

#define GPIO_KBD_OUT_1 GPIOC
#define GPIO_KBD_OUT_2 GPIOD
#define GPIO_KBD_OUT_3 GPIOD
#define GPIO_KBD_OUT_4 GPIOD
#define GPIO_KBD_OUT_5 GPIOD
#define GPIO_KBD_OUT_6 GPIOC

GPIO_TypeDef *GPIO_KBD_OUT[6] = {GPIO_KBD_OUT_1, GPIO_KBD_OUT_2,
                                 GPIO_KBD_OUT_3, GPIO_KBD_OUT_4,
                                 GPIO_KBD_OUT_5, GPIO_KBD_OUT_6};

#define PIN_KBD_OUT_1 GPIO_Pin_0
#define PIN_KBD_OUT_2 GPIO_Pin_0
#define PIN_KBD_OUT_3 GPIO_Pin_4
#define PIN_KBD_OUT_4 GPIO_Pin_3
#define PIN_KBD_OUT_5 GPIO_Pin_2
#define PIN_KBD_OUT_6 GPIO_Pin_7

uint16_t PIN_KBD_OUT[6] = {PIN_KBD_OUT_1, PIN_KBD_OUT_2, PIN_KBD_OUT_3,
                           PIN_KBD_OUT_4, PIN_KBD_OUT_5, PIN_KBD_OUT_6};

const uint8_t KBD_EVENT_PRESSED = 0b01000000;
const uint8_t KBD_EVENT_RELEASED = 0b00000000;
const uint8_t KBD_EVENT_PING = 0b10000000;

bool keyboard_data[6][6];

bool temp_keyboard_data[6][6];

void KeyboardInit() {
  _gpio_init(GPIO_KBD_IN_1, PIN_KBD_IN_1, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
  _gpio_init(GPIO_KBD_IN_2, PIN_KBD_IN_2, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
  _gpio_init(GPIO_KBD_IN_3, PIN_KBD_IN_3, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
  _gpio_init(GPIO_KBD_IN_4, PIN_KBD_IN_4, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
  _gpio_init(GPIO_KBD_IN_5, PIN_KBD_IN_5, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
  _gpio_init(GPIO_KBD_IN_6, PIN_KBD_IN_6, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);

  _gpio_init(GPIO_KBD_OUT_1, PIN_KBD_OUT_1, GPIO_Mode_IPD, GPIO_Speed_50MHz);
  _gpio_init(GPIO_KBD_OUT_2, PIN_KBD_OUT_2, GPIO_Mode_IPD, GPIO_Speed_50MHz);
  _gpio_init(GPIO_KBD_OUT_3, PIN_KBD_OUT_3, GPIO_Mode_IPD, GPIO_Speed_50MHz);
  _gpio_init(GPIO_KBD_OUT_4, PIN_KBD_OUT_4, GPIO_Mode_IPD, GPIO_Speed_50MHz);
  _gpio_init(GPIO_KBD_OUT_5, PIN_KBD_OUT_5, GPIO_Mode_IPD, GPIO_Speed_50MHz);
  _gpio_init(GPIO_KBD_OUT_6, PIN_KBD_OUT_6, GPIO_Mode_IPD, GPIO_Speed_50MHz);
}

void KeyboardScan() {
  for (size_t i = 0; i < 6; i++) {
    GPIO_WriteBit(GPIO_KBD_IN[i], PIN_KBD_IN[i], Bit_RESET);
  }
  for (size_t i = 0; i < 6; i++) {
    GPIO_WriteBit(GPIO_KBD_IN[i], PIN_KBD_IN[i], Bit_SET);
    Delay_Us(1);
    for (size_t j = 0; j < 6; j++) {
      keyboard_data[i][j] =
          (bool)(GPIO_ReadInputDataBit(GPIO_KBD_OUT[j], PIN_KBD_OUT[j]));
    }
    GPIO_WriteBit(GPIO_KBD_IN[i], PIN_KBD_IN[i], Bit_RESET);
    Delay_Us(1);
  }
}

void serialInit() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);
  // UART1: PD5 tx, PD6 rx
  _gpio_init(GPIOD, GPIO_Pin_5, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
  _gpio_init(GPIOD, GPIO_Pin_6, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
  // struct
  USART_InitTypeDef USART_InitStructure = {0};
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_9b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE);
}

void SendKeyboardEvent(uint8_t key_code, uint8_t event) {
  uint8_t data = key_code | event;
  putchar(data);
}

void IWDG_Feed_Init(u16 prer, u16 rlr) {
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(prer);
  IWDG_SetReload(rlr);
  IWDG_ReloadCounter();
  IWDG_Enable();
}

void writeBit(uint8_t *x, size_t i, uint8_t v) {
  if (v == 0) {
    *x &= ~(1 << i);
  } else {
    *x |= (1 << i);
  }
}

void sendKeyInfo(uint8_t x) {
  if (x > 5) {
    x = 5;
  }
  uint8_t r = 0;
  for (size_t j = 0; j < 6; j++) {
    writeBit(&r, j, keyboard_data[x][j]);
  }
  USART_SendData(USART1, r);
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
  }
  Delay_Us(100);
}

void processCommand() {
  if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET) {
    uint8_t u1_dt = USART_ReceiveData(USART1);
    if (u1_dt == 123) {
      sendKeyInfo(0);
      sendKeyInfo(1);
      sendKeyInfo(2);
      sendKeyInfo(3);
      sendKeyInfo(4);
      sendKeyInfo(5);
    }
    USART_ClearFlag(USART1, USART_FLAG_RXNE);
  }
}

int MainProcess() {
  while (1) {
    KeyboardScan();
    processCommand();
    Delay_Us(1);
    IWDG_ReloadCounter();
  }
  for (;;) {
  }
}

int main() {
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  SystemCoreClockUpdate();
  Delay_Init();
  IWDG_Feed_Init(IWDG_Prescaler_128, 500);
  // gpio
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  // ---
  // init
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      keyboard_data[i][j] = false;
      temp_keyboard_data[i][j] = false;
    }
  }
  KeyboardInit();
  // USART_Printf_Init(115200);
  serialInit();
  IWDG_ReloadCounter();
  // ---
  return MainProcess();
}

void HardFault_Handler(void) {
  while (1) {
  }
}
