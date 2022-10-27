/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
void CheckCharMatch(UART_HandleTypeDef *hUart);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_Pin GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOC
#define BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define LCD_RST_Pin GPIO_PIN_0
#define LCD_RST_GPIO_Port GPIOC
#define LCD_CE_Pin GPIO_PIN_1
#define LCD_CE_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_4
#define LCD_DC_GPIO_Port GPIOC
#define BLE_CMD_Pin GPIO_PIN_6
#define BLE_CMD_GPIO_Port GPIOC
#define BLE_PWR_Pin GPIO_PIN_8
#define BLE_PWR_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
extern TIM_HandleTypeDef htim6;
#define BluCmd(_a)		HAL_GPIO_WritePin(BLE_CMD_GPIO_Port, BLE_CMD_Pin, (GPIO_PinState)_a)
#define BluPwr(_a)		HAL_GPIO_WritePin(BLE_PWR_GPIO_Port, BLE_PWR_Pin, (GPIO_PinState)_a)
#define F4_CS(_a)		HAL_GPIO_WritePin(F4_CS_GPIO_Port, F4_CS_Pin, (GPIO_PinState)_a)
#define rele1(_a)		HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, (GPIO_PinState)!_a)
#define rele2(_a)		HAL_GPIO_WritePin(RELE2_GPIO_Port, RELE2_Pin, (GPIO_PinState)!_a)
#define rele3(_a)		HAL_GPIO_WritePin(RELE3_GPIO_Port, RELE3_Pin, (GPIO_PinState)!_a)
#define rele4(_a)		HAL_GPIO_WritePin(RELE4_GPIO_Port, RELE4_Pin, (GPIO_PinState)!_a)
#define IN(_a)			((uint8_t) (HAL_GPIO_ReadPin(IN2_GPIO_Port,_a)))
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
