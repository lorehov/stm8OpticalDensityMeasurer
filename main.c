/* MAIN.C file
 * 
 * Copyright (c) 2002-2005 STMicroelectronics
 */
#include "main.h"

int main()
{
    int result = 0;
	Periph_Init();
	while (1) {
        result = Conversion_Make();
        Concentration_Check(result);
        Interaction_Do(result);
        DelayMS(4);
        
        
	}
	return 0;
}

void Periph_Init()
{
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
	GPIO_DeInit(GPIOB);
   	GPIO_DeInit(GPIOE);
	GPIO_Init(GPIOB, GPIO_PIN_ALL, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_Init(GPIOB, GPIO_PIN_ALL, GPIO_MODE_OUT_PP_LOW_FAST);
    
	ADC2_DeInit();
	ADC2_Init(ADC2_CONVERSIONMODE_CONTINUOUS, ADC2_CHANNEL_8,
						ADC2_PRESSEL_FCPU_D8, ADC2_EXTTRIG_GPIO, DISABLE,
						ADC2_ALIGN_RIGHT, ADC2_SCHMITTTRIG_CHANNEL8, 	 
						DISABLE);
	ADC2_Cmd(ENABLE);
    ADC2_StartConversion();

	UART1_DeInit();
	UART1_Init(9600, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, 
						 UART1_PARITY_NO, UART1_SYNCMODE_CLOCK_DISABLE, 
						 UART1_MODE_TXRX_ENABLE);
}

void DelayMS(int n)
{
	int i;
    int count = n*100;
	for (i=0; i<count; i++){
	}
}

int Conversion_Make()
{
    int con_val_1 = 0;
    int con_val_2 = 0;
    int con_res = 0;

    GPIO_WriteHigh(GPIOB, GPIO_PIN_6);
    DelayMS(1);
    con_val_1 = ADC2_GetConversionValue();
    DelayMS(4);
    GPIO_WriteLow(GPIOB, GPIO_PIN_6);
    DelayMS(1);
    con_val_2 = ADC2_GetConversionValue();
    con_res = (con_val_1 - con_val_2);
    if (con_val_1 > con_val_2) {
        con_res = con_val_1 - con_val_2;
    } else {
        con_res = con_val_2 - con_val_1;
    }
    return (int)con_res;
}

void Concentration_Check(int con)
{
    if (con >= treshhold) {
        GPIO_WriteHigh(GPIOE, GPIO_PIN_1);
    } else {
        GPIO_WriteLow(GPIOE, GPIO_PIN_1);
    }
}

void Interaction_Do(int result)
{
    if (RESET != UART1_GetFlagStatus(UART1_FLAG_RXNE)) {
        if (state == RECEIVING_TRESHHOLD) {
            Treshhold_Receive();
            return;
        }
        
        state = UART1_ReceiveData8();
        switch(state) {
            case TRANSMITTING_DATA: Data_Send(result); break;
            case TRANSMITTING_TRESHHOLD: Data_Send(treshhold); break;
            case RECEIVING_TRESHHOLD: break;
            default: Data_Send(result); break;
        }
    }
}

void Data_Send(int d)
{
    UART1_SendData8(d);
    while (UART1_GetFlagStatus(UART1_FLAG_TC) == RESET){
    }
}

void Treshhold_Receive()
{
    treshhold = UART1_ReceiveData8();
    UART1_ClearFlag(UART1_FLAG_TXE);
    state = TRANSMITTING_DATA;
    Data_Send(1);
}