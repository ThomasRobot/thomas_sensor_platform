/**
 *	External interrupts example
 *
 *	@author 	Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.com
 *	@ide		Keil uVision 5
 */
#include "stm32f4xx.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

uint32_t counter = 0;
int send_gps = 0;
int gps_pulse_sent = 0;

#define USART_TXEMPTY(USARTx)               ((USARTx)->SR & USART_FLAG_TXE)
#define USART_WAIT(USARTx)                  do { while (!USART_TXEMPTY(USARTx)); } while (0)
void uputs(char* str) {
	/* Go through entire string */
	while (*str) {
		/* Wait to be ready, buffer empty */
		USART_WAIT(USART2);
		/* Send data */
		USART2->DR = (uint16_t)(*str++ & 0x01FF);
		/* Wait to be ready, buffer empty */
		USART_WAIT(USART2);
	}
}

/* Configure pins to be interrupts */
void InitEXTI(void) {
	/* Set variables used */
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	/* Enable clock for GPIOD */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* Enable clock for SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Common pin settings */
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	/* Set pin as input */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* Set pin as output */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_13;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* Tell system that you will use PD0 for EXTI_Line0 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);

	/* PD0 is connected to EXTI_Line0 */
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	/* Enable interrupt */
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	/* Interrupt mode */
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStruct);

	/* Add IRQ vector to NVIC */
	/* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStruct);
}

void InitUsart()
{
	// Enable clock for GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	/**
	 * Tell pins P and PB7 which alternating function you will use
	 * @important Make sure, these lines are before pins configuration!
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	// Initialize pins as alternating function
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	/**
	 * Set Baudrate to value you pass to function
	 * Disable Hardware Flow control
	 * Set Mode To TX and RX, so USART will work in full-duplex mode
	 * Disable parity bit
	 * Set 2 stop bit
	 * Set Data bits to 8
	 *
	 * Initialize USART1
	 * Activate USART1
	 */
	USART_InitTypeDef USART_InitStruct;
	USART_InitStruct.USART_BaudRate = 230400;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = /*USART_Mode_Tx | */USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStruct);
	USART_Cmd(USART1, ENABLE);

	/**
	 * Set Channel to USART1
	 * Set Channel Cmd to enable. That will enable USART1 channel in NVIC
	 * Set Both priorities to 0. This means high priority
	 *
	 * Initialize NVIC
	 */
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);

	/**
	 * Enable RX interrupt
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void InitUsart2()
{
	// Enable clock for GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	/**
	 * Tell pins P and PB7 which alternating function you will use
	 * @important Make sure, these lines are before pins configuration!
	 */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);


	// Initialize pins as alternating function
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);


	/**
	 * Set Baudrate to value you pass to function
	 * Disable Hardware Flow control
	 * Set Mode To TX and RX, so USART will work in full-duplex mode
	 * Disable parity bit
	 * Set 2 stop bit
	 * Set Data bits to 8
	 *
	 * Initialize USART1
	 * Activate USART1
	 */
	USART_InitTypeDef USART_InitStruct;
	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &USART_InitStruct);
	USART_Cmd(USART2, ENABLE);
}


/* Handle PD0 interrupt */
void EXTI0_IRQHandler(void) {
	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {

		/* For camera: 25Hz */
		if (((counter+1) % 16) == 0) {
			GPIO_SetBits(GPIOD, GPIO_Pin_1);
		} else if (((counter+1) % 16) == 1) {
			GPIO_ResetBits(GPIOD, GPIO_Pin_1);
		}

		/* For LiDAR: 1Hz */
		if (((counter+1) % 400) == 0) {
			GPIO_ResetBits(GPIOD, GPIO_Pin_2);
			// GPIO_SetBits(GPIOD, GPIO_Pin_13); // For debug
			gps_pulse_sent = 1;
		} else if (((counter+1) % 400) == 10) {
			GPIO_SetBits(GPIOD, GPIO_Pin_2);
			// GPIO_ResetBits(GPIOD, GPIO_Pin_13); // For debug
		}

		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

uint8_t buf[300];
uint16_t buf_p = 0;
uint16_t data_len = 0;
uint8_t dd = 0;
void USART1_IRQHandler(void) {
	/* Check if interrupt was because data is received */
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
		uint8_t data = USART_ReceiveData(USART1);

		dd = data;

		if (buf_p >= 300) {
			buf_p = 0;
			return;
		} else {
			buf[buf_p++] = data;
		}

		if (buf_p == 1) {
			if (data != 0xfa) {
				buf_p = 0;
			}
		} else if (buf_p == 2) {
			if (data != 0xff) {
				buf_p = 0;
			}
		} else if (buf_p == 4) {
			data_len = data;
		} else if (buf_p == (5 + data_len)) { // Check it
			uint16_t sum = 0;
			for (int i = 1; i < buf_p; ++i)
				sum += buf[i];
			if ((sum & 0x00ff) == 0) {
				// checksum pass
				if (buf[2] == 0x36) {

					if (data_len >= 5) {
						if (buf[4] == 0x10 && buf[5] == 0x20 && buf[6] == 2) {

							static uint32_t short_counter = 0;
							static uint32_t loop = 0;
							uint32_t new_short_counter = buf[7] << 8 | buf[8];
							if (new_short_counter < short_counter)
								loop++;
							short_counter = new_short_counter;

							counter = loop << 16 | short_counter;

							if (gps_pulse_sent) {
								gps_pulse_sent = 0;
								send_gps = 1;
							}
							/*
							static int hh = 0;
							static int mm = 0;
							static int ss = 0;

							hhh = hh;
							mmm = mm;
							sss = ss;
							send_gps = 1;

							ss++;
							if (ss == 60) {
								ss = 0;
								mm++;
								if (mm == 60) {
									mm = 0;
									hh++;
								}
							}
							*/
#if 0
							char tt[10];
							itoa(counter, tt, 10);
							uputs("C:");
							uputs(tt);
							uputs("\r\n");
#endif
						} else {
							uputs("No TS?\r\n");
						}

					} else {
						uputs("Too short MTData2\r\n");
					}
				}
			} else {
				uputs("Checksum failed\r\n");
			}
			buf_p = 0;
		} else if (buf_p > (5 + data_len)) {
			buf_p = 0;
			uputs("WTF?\r\n");
		}
	}
}

int main(void) {

	/* System init */
	SystemInit();
	/* Configure PD0 as interrupt */
	InitEXTI();
	/* Configure PB6/7 as usart port */
	InitUsart();
	InitUsart2();

	uputs("Hello world\r\n");

	char content[] = "$GPRMC,000000,A,hh.mm,N,ss.00,W,3.3,4.4,010117,004.2,W*";

	// char tmpc[2]; tmpc[1] = '\0';

	while (1) {
		if (send_gps) {
			send_gps = 0;

			int hh, mm, ss;
			ss = counter / 400;
			mm = ss / 60;
			ss = ss % 60;
			hh = mm / 60;
			mm = mm % 60;

			content[ 7] = content[16] = '0' + hh / 10;
			content[ 8] = content[17] = '0' + hh % 10;
			content[ 9] = content[19] = '0' + mm / 10;
			content[10] = content[20] = '0' + mm % 10;
			content[11] = content[24] = '0' + ss / 10;
			content[12] = content[25] = '0' + ss % 10;

			char cs;
			cs = 0x00;
			for (int i = 1; i < strlen(content)-1; ++i) {
				cs ^= content[i];
			}

			char cs_str[3];
			itoa(cs, cs_str, 16);

			uputs(content);
			uputs(cs_str);
			uputs("\r\n");
		}

		/*
		if (dd != 0) {
			tmpc[0] = dd;
			uputs(tmpc);
			dd = 0;
		}
		 */
	}
}
