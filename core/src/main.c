/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "functionalTest.h"
#include "test.h"
#include "bmarkTest.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(int argc, char **argv)
{
  /* USER CODE BEGIN 1 */
  uint8_t counter = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  // set up GPIO registers
  //GPIO_InitTypeDef GPIO_init_config; // in example chip for some reason
  //GPIO_init_config.mode = GPIO_MODE_OUTPUT;
  //GPIO_init_config.pull = GPIO_PULL_NONE;
  //GPIO_init_config.drive_strength = GPIO_DS_STRONG;
  /**PLL_InitTypeDef PLL_init_config;
  PLL_init_config.div = 1;
  PLL_init_config.mul = 1;*/
  //HAL_GPIO_init(GPIOA, &GPIO_init_config, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
  /*HAL_PLL_init(PLL, &PLL_init_config, 1, 1);*/
  

  // set up UART registers
  //UART_InitTypeDef UART_init_config;
  //UART_init_config.baudrate = 115200;
  //UART_init_config.mode = UART_MODE_TX_RX;
  //UART_init_config.stopbits = UART_STOPBITS_2;
  //HAL_UART_init(UART0, &UART_init_config);
  SET_BITS(UART0->TXCTRL, UART_TXCTRL_TXEN_MSK);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint64_t mhartid = READ_CSR("mhartid");
    
  }
  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/*
 * Main function for secondary harts
 *
 * Multi-threaded programs should provide their own implementation.
 */
void __attribute__((noreturn)) __main(void)
{
  uint8_t counter = 0;
  while (1)
  {
    uint64_t mhartid = READ_CSR("mhartid");
    printf("Hello world from hart %ld: %d\n", mhartid, counter);
    counter += 1;
    /* USER CODE END WHILE */
    printf("=========Start Simple test=========\n");
    simple_functional_test();
    printf("=========Start benchmark test=========\n");
    bmark_test();
    //printf("=========Start main test=========\n");
    //main_test();
  }
}
