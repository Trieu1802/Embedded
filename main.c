#include "MKL46Z4.h"
#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "pin_mux.h"
#include "board.h"
#include "lcd.h"

#define M_PI 															(3.14159265358979323846)

#define STATE_STANDBY														(0)
#define STATE_ACTIVE														(1)

#define MAG_I2C_ADDR 													(0x0E)
#define MAG_OUT_REG_START 										(0x01)
#define MAG_OUT_DATA_LENGTH										(0x06)

#define MAG_CTRL_REG1 												(0x10)
#define MAG_CTRL_REG2 												(0x11)

#define MAG_CTRL_REG2_AUTO_RESET_MODE 				(0x80)
#define MAG_CTRL_REG1_STANDBY_MODE 						(0x00)
#define MAG_CTRL_REG1_ACTIVE_MODE 						(0x01)

#define GREEN_LED_PIN          (1<<5)
#define RED_LED_PIN          	(1<<29)
#define SW1_PIN                (1<<3)
#define SW2_PIN                (1<<12)

#define PORTC_D_IRQ_NBR (IRQn_Type) 31

#define GREEN_LED_TICKS_TO_CHANGE	(500U)
#define RED_LED_TICKS_TO_CHANGE	(250U)

void init_i2c(void);
void send_i2c(uint8_t device_addr, uint8_t reg_addr, uint8_t value);
void read_i2c(uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);
void init_mag();
void PORTC_PORTD_IRQHandler(void);
void SysTick_Handler (void);
void Delay (uint32_t TICK);
void init_SysTick_interrupt();
void ledBlink ();
void init_GPIO ();
void PORTC_PORTD_IRQHandler();
uint16_t mag_calculator();

int32_t volatile msTicks = 0;
int32_t volatile redTicks = 0;
int32_t volatile greenTicks = 0;
bool state = STATE_ACTIVE;
uint8_t rx_buff[MAG_OUT_DATA_LENGTH];

int main(void)
{
	init_i2c();
	init_GPIO();
	init_SysTick_interrupt();
	LCD_Init();
	init_mag();

	while (1)
	{
		ledBlink();
		if (state == STATE_STANDBY) continue;
		uint16_t angle = mag_calculator();
		LCD_DisplayDemical(angle);
		Delay(10);
	}
}

uint16_t mag_calculator()
{
	read_i2c(MAG_I2C_ADDR, MAG_OUT_REG_START, rx_buff, MAG_OUT_DATA_LENGTH);
		
	int16_t x_raw = (int16_t) ((((uint16_t) rx_buff[0]) << 8U)| rx_buff[1]);
	int16_t y_raw = (int16_t) ((((uint16_t) rx_buff[2]) << 8U)| rx_buff[3]);
	double_t angle = ((double_t)(atan2(y_raw, x_raw)))*180/M_PI;
	if (angle < 0) {
		angle += 360.0;
  	}
	return (uint16_t) angle;
}

void init_mag()
{
	send_i2c(MAG_I2C_ADDR, MAG_CTRL_REG1, MAG_CTRL_REG1_STANDBY_MODE);
	send_i2c(MAG_I2C_ADDR, MAG_CTRL_REG2, MAG_CTRL_REG2_AUTO_RESET_MODE);
	send_i2c(MAG_I2C_ADDR, MAG_CTRL_REG1, MAG_CTRL_REG1_ACTIVE_MODE);
}

void init_i2c()
{
	i2c_master_config_t m_config;
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	BOARD_I2C_ConfigurePins();

	I2C_MasterGetDefaultConfig(&m_config);
	I2C_MasterInit(I2C0, &m_config, CLOCK_GetFreq(I2C0_CLK_SRC));
}

void send_i2c(uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
	i2c_master_transfer_t frame_i2c;

	frame_i2c.slaveAddress = device_addr;
	frame_i2c.direction = kI2C_Write;
	frame_i2c.subaddress = (uint32_t)reg_addr;
	frame_i2c.subaddressSize = 1;
	frame_i2c.data = &value;
	frame_i2c.dataSize = 1;
	frame_i2c.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferBlocking(I2C0, &frame_i2c);
}

void read_i2c(uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
	i2c_master_transfer_t frame_i2c;

	frame_i2c.slaveAddress = device_addr;
	frame_i2c.direction = kI2C_Read;
	frame_i2c.subaddress = (uint32_t)reg_addr;
	frame_i2c.subaddressSize = 1;
	frame_i2c.data = rxBuff;
	frame_i2c.dataSize = rxSize;
	frame_i2c.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferBlocking(I2C0, &frame_i2c);
}

void init_GPIO()
{
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK; /* enable clock to Port D */
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; /* enable clock to Port C */
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; /* enable clock to Port E */
	
	PORTD->PCR[5] = PORT_PCR_MUX(1); /* make PTD5 pin as GPIO */
	PORTE->PCR[29] = PORT_PCR_MUX(1); /* make PTE29 pin as GPIO */
	
	PORTC->PCR[3]= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;; /* make PTC3 pin as GPIO and enable pull-up resistor */
	PORTC->PCR[12]= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;; /* make PTC12 pin as GPIO and enable pull-up resistor */
	
	PORTC->PCR[3] |= PORT_PCR_IRQC(0x0A);/* thiet lap ngat suon xuong cho pin PORTC3 */
	PORTC->PCR[12] |= PORT_PCR_IRQC(0x0A);/* thiet lap ngat suon xuong cho pin PORTC12 */
	NVIC_ClearPendingIRQ(PORTC_D_IRQ_NBR);/* xoa cac interrupt request dang ton tai o port C va D */
	NVIC_EnableIRQ(PORTC_D_IRQ_NBR);/* enable interrupt request cho port C va D */
	
	PTD->PDDR |= GREEN_LED_PIN; /* make PTD5 as output pin */
	PTE->PDDR |= RED_LED_PIN; /* make PTE29 as output pin */
}	

void init_SysTick_interrupt()
{
	SysTick->LOAD = SystemCoreClock / 1000;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk;
	NVIC_EnableIRQ(SysTick_IRQn);

}

void SysTick_Handler (void)
{
	msTicks ++;
	redTicks ++;
	greenTicks ++;
}

void ledBlink ()
{
	if (state == STATE_STANDBY)
	{
		PTD->PSOR |= GREEN_LED_PIN;
		if (redTicks > RED_LED_TICKS_TO_CHANGE)
		{
			PTE->PTOR |= RED_LED_PIN;
			redTicks = 0;
		}
	}
	else 
	{
		PTE->PSOR |= RED_LED_PIN;
		if (greenTicks > GREEN_LED_TICKS_TO_CHANGE)
		{
			PTD->PTOR |= GREEN_LED_PIN;
			greenTicks = 0;
		}
	}
}

void PORTC_PORTD_IRQHandler()
{
	if ((PTC->PDIR & SW2_PIN) == 0)
	{
		while ((PTC->PDIR & SW2_PIN) == 0);
		state = STATE_ACTIVE;
		PORTC->PCR[12] |= PORT_PCR_ISF_MASK;
	}
	if ((PTC->PDIR & SW1_PIN) == 0)
	{
		while ((PTC->PDIR & SW1_PIN) == 0);
		state = !state;
		PORTC->PCR[3] |= PORT_PCR_ISF_MASK;
	}
}

void Delay (uint32_t TICK)
{
	while (msTicks < TICK);
	msTicks = 0;
}