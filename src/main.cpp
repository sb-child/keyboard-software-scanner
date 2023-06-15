#include <ch32v00x.h>
#include <debug.h>
#include <stdio.h>

// void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void Delay_Init(void);
void Delay_Ms(uint32_t n);
void Delay_Us(uint32_t n);

inline void _gpio_init(GPIO_TypeDef *x, uint16_t pin, GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed)
{
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

GPIO_TypeDef *GPIO_KBD_IN[6] = {
    GPIO_KBD_IN_1, GPIO_KBD_IN_2, GPIO_KBD_IN_3, GPIO_KBD_IN_4, GPIO_KBD_IN_5, GPIO_KBD_IN_6};

#define PIN_KBD_IN_1 GPIO_Pin_1
#define PIN_KBD_IN_2 GPIO_Pin_2
#define PIN_KBD_IN_3 GPIO_Pin_3
#define PIN_KBD_IN_4 GPIO_Pin_4
#define PIN_KBD_IN_5 GPIO_Pin_5
#define PIN_KBD_IN_6 GPIO_Pin_6

uint16_t PIN_KBD_IN[6] = {
    PIN_KBD_IN_1, PIN_KBD_IN_2, PIN_KBD_IN_3, PIN_KBD_IN_4, PIN_KBD_IN_5, PIN_KBD_IN_6};

#define GPIO_KBD_OUT_1 GPIOC
#define GPIO_KBD_OUT_2 GPIOD
#define GPIO_KBD_OUT_3 GPIOD
#define GPIO_KBD_OUT_4 GPIOD
#define GPIO_KBD_OUT_5 GPIOD
#define GPIO_KBD_OUT_6 GPIOC

GPIO_TypeDef *GPIO_KBD_OUT[6] = {
    GPIO_KBD_OUT_1, GPIO_KBD_OUT_2, GPIO_KBD_OUT_3, GPIO_KBD_OUT_4, GPIO_KBD_OUT_5, GPIO_KBD_OUT_6};

#define PIN_KBD_OUT_1 GPIO_Pin_0
#define PIN_KBD_OUT_2 GPIO_Pin_0
#define PIN_KBD_OUT_3 GPIO_Pin_4
#define PIN_KBD_OUT_4 GPIO_Pin_3
#define PIN_KBD_OUT_5 GPIO_Pin_2
#define PIN_KBD_OUT_6 GPIO_Pin_7

uint16_t PIN_KBD_OUT[6] = {
    PIN_KBD_OUT_1, PIN_KBD_OUT_2, PIN_KBD_OUT_3, PIN_KBD_OUT_4, PIN_KBD_OUT_5, PIN_KBD_OUT_6};

const uint8_t KBD_EVENT_PRESSED = 0b01000000;
const uint8_t KBD_EVENT_RELEASED = 0b00000000;
const uint8_t KBD_EVENT_PING = 0b10000000;

bool keyboard_data[6][6];

bool temp_keyboard_data[6][6];

uint32_t scan_passed_count = 0;

void KeyboardInit()
{
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

void KeyboardScan()
{
    for (size_t i = 0; i < 6; i++)
    {
        GPIO_WriteBit(GPIO_KBD_IN[i], PIN_KBD_IN[i], Bit_RESET);
    }
    for (size_t i = 0; i < 6; i++)
    {
        GPIO_WriteBit(GPIO_KBD_IN[i], PIN_KBD_IN[i], Bit_SET);
        for (size_t j = 0; j < 6; j++)
        {
            keyboard_data[i][j] = (bool)(GPIO_ReadInputDataBit(GPIO_KBD_OUT[j], PIN_KBD_OUT[j]));
        }
        GPIO_WriteBit(GPIO_KBD_IN[i], PIN_KBD_IN[i], Bit_RESET);
    }
}

void SendKeyboardEvent(uint8_t key_code, uint8_t event)
{
    uint8_t data = key_code | event;
    putchar(data);
}

void IWDG_Feed_Init(u16 prer, u16 rlr)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(prer);
    IWDG_SetReload(rlr);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

int MainProcess()
{
    while (1)
    {
        KeyboardScan();
        scan_passed_count++;
        for (size_t i = 0; i < 6; i++)
        {
            for (size_t j = 0; j < 6; j++)
            {
                if (keyboard_data[i][j] != temp_keyboard_data[i][j])
                {
                    SendKeyboardEvent(i * 6 + j, keyboard_data[i][j] ? KBD_EVENT_PRESSED : KBD_EVENT_RELEASED);
                    scan_passed_count = 0;
                }
                temp_keyboard_data[i][j] = keyboard_data[i][j];
            }
        }
        if (scan_passed_count > 1200)
        {
            // SendKeyboardEvent(0, KBD_EVENT_PING);
            for (size_t i = 0; i < 6; i++)
            {
                for (size_t j = 0; j < 6; j++)
                {
                    SendKeyboardEvent(i * 6 + j, keyboard_data[i][j] ? KBD_EVENT_PRESSED : KBD_EVENT_RELEASED);
                }
            }
            scan_passed_count = 0;
        }
        IWDG_ReloadCounter();
    }
    for (;;)
    {
    }
}

int main()
{
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
    for (size_t i = 0; i < 6; i++)
    {
        for (size_t j = 0; j < 6; j++)
        {
            keyboard_data[i][j] = false;
            temp_keyboard_data[i][j] = false;
        }
    }
    KeyboardInit();
    USART_Printf_Init(115200);
    IWDG_ReloadCounter();
    // ---
    return MainProcess();
}

void HardFault_Handler(void)
{
    while (1)
    {
    }
}
