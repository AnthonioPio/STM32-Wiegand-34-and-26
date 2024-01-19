/*
 * wiegand_handler.h
 *
 *  Created on: Dec 27, 2023
 *      Author: Anthonio Pio
 *
 *      Usage: In Code
 *			* This Code is implimented int CPP.
 *				- Rename main.c to main.cpp to use
 *      	* Add GPIO EXTI for D0 and D1.
 *      	* Add Timer for timeout (configured to trigger callback every 1ms).
 *      	* Add Callback "void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)", for EXTI Event.
 *      		- When EXTI callback occurs, call function 'exti_callback(uint16_t pin);' to process Wiegand Data.
 *      	* Add Callback "void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)" for TIMER Event.
 *      		- When TIMER callback occurs, call function 'timer_callback(TIM_HandleTypedef *htim). to start the timeout.
 *
 *
 */

#ifndef INC_INTERFACE_WIEGAND_HANDLER_H_
#define INC_INTERFACE_WIEGAND_HANDLER_H_

/*********************************************** INCLUDES ***********************************************/
#include <stm32f4xx_hal.h>
#include <stdio.h>
#include <stdint.h>

/*********************************************** DEFINITIONS ***********************************************/
#define WIEGAND_LENGTH_26_BITS		(uint8_t)26
#define WIEGAND_LENGTH_34_BITS		(uint8_t)34


/*********************************************** TYPEDEF STRUCT ***********************************************/
typedef struct
{
	uint32_t l = 0;
	uint32_t h = 0;
}wiegand_output_struct;

typedef struct
{
	bool					ready;
	uint8_t 				counter;
	uint32_t				d0_l;
	uint32_t				d0_h;
	uint32_t				d1_l;
	uint32_t				d1_h;
	uint8_t					lpb;
	uint8_t					tpb;
	uint32_t				fac_code;
	uint32_t				card_id;
	uint32_t				last_fac_code;
	uint32_t				last_card_id;
}wiegand_data_struct;

typedef struct
{
	TIM_HandleTypeDef 		*htim;
	uint32_t				limit;
	uint32_t				counter;
}wiegand_timer_struct;

typedef struct
{
	uint8_t					length;
	uint16_t				d0_pin;
	uint16_t				d1_pin;
}wiegand_config_struct;

class WIEGAND
{
public:
	wiegand_config_struct 	config;
	wiegand_timer_struct	timer;
	wiegand_data_struct 	data;

	WIEGAND(uint8_t length, uint16_t d0_pin, uint16_t d1_pin, TIM_HandleTypeDef *htim, uint32_t timeout)
	{
		config.length 		= length;
		config.d0_pin 		= d0_pin;
		config.d1_pin 		= d1_pin;
		timer.htim 			= htim;
		timer.limit 		= timeout;
	}

	void initialize(void)
	{
		reset();
		HAL_TIM_Base_Start_IT(timer.htim);
	}

	void exti_callback(uint16_t pin)
	{
		if(pin == config.d0_pin || pin == config.d1_pin)
		{
			if(data.counter < config.length)
			{
				if(pin == config.d0_pin)
				{
					if(data.counter < 32)
					{
						data.d0_h |= 1 << data.counter;
					}
					else if(data.counter >= 32)
					{
						data.d0_l |= 1 << (data.counter - 32);
					}
				}
				else if(pin == config.d1_pin)
				{
					if(data.counter < 32)
					{
						data.d1_h |= 1 << data.counter;
					}
					else if(data.counter >= 32)
					{
						data.d1_l |= 1 << (data.counter - 32);
					}
				}
				data.counter++;
			}
			if(data.counter >= (config.length))
			{
				data.ready = true;
				timer.counter = 0;
			}
		}
	}

	void start(void)
	{
		if(data.ready)
		{
			wiegand_output_struct output = {0};
			int b = 0;
			int c = 0;
			switch(config.length)
			{
				case WIEGAND_LENGTH_26_BITS:
					for(int a = 25; a > 0; a--)
					{
						if(data.d1_h & (1 << a)) output.l |= 1 << b;
						b++;
					}
					if(output.l & 0x00000001UL) data.tpb = 1;
					else data.tpb = 0;
					if(output.l & 0x02000000UL) data.lpb = 1;
					else data.lpb = 0;
					data.fac_code = (output.l) & 0x0000FFFFUL;
					data.card_id = (output.l >> 1) & 0x0000FFFFUL;
					break;
				case WIEGAND_LENGTH_34_BITS:
					for(int a = 31; a >= 0; a--)
					{
						if(data.d1_h & (1 << a)) output.h |= 1 << b;
						b++;
					}
					for(int a = 1; a >= 0; a--)
					{
						if(data.d1_l & (1 << a)) output.l |= 1 << c;
						c++;
					}
					if(output.l & 0x00000001UL) data.tpb = 1;
					else data.tpb = 0;
					if(output.h & 0x80000000UL) data.lpb = 1;
					else data.lpb = 0;
					data.fac_code = (output.h >> 15);
					data.card_id = ((output.h << 1) | ((output.l & 0x0002) >> 1));
					break;
			}
			if(!(check_parity(output.l, output.h)))
			{
				reset();
				return;
			}
		}
		if(timer.counter >= timer.limit && (!data.ready))
		{
			reset();
		}
	}

	bool check_parity(uint32_t data_l, uint32_t data_h)
	{
		bool status = false;
		uint16_t lpb = 0;
		uint16_t tpb = 0;
		uint16_t data_e = 0;
		uint16_t data_o = 0;
		switch(config.length)
		{
			case WIEGAND_LENGTH_26_BITS:
				data_e = ((data_l >> 13) & 0x0FFF);
				data_o = ((data_l >> 1) & 0x0FFF);
				lpb = data_e % 1;
				tpb = data_o % 2;
				break;
			case WIEGAND_LENGTH_34_BITS:
				data_e = ((data_h >> 15) & 0x0000FFFFUL);
				data_o = ((data_h << 1) | ((data_l & 0x0002) >> 1));
				lpb = data_e % 1;
				tpb = data_o % 2;
				break;
		}
		if((data.lpb == lpb) && (data.tpb == tpb)) status = true;
		data.last_fac_code = data.fac_code;
		data.last_card_id = data.card_id;
		return status;
	}

	void reset(void)
	{
		data.counter 	= 0;
		data.d0_l 		= 0;
		data.d0_h 		= 0;
		data.d1_l 		= 0;
		data.d1_h 		= 0;
		data.lpb		= 0;
		data.tpb		= 0;
		data.fac_code	= 0;
		data.card_id	= 0;
		data.ready 		= false;
		timer.counter 	= 0;
	}

	void timer_callback(TIM_HandleTypeDef *htim)
	{
		if(!(timer.htim->Instance == htim->Instance)) return;
		if(data.counter == 0) return;
		if(timer.counter > timer.limit) reset();
		else timer.counter++;
	}
};



#endif /* INC_INTERFACE_WIEGAND_HANDLER_H_ */
