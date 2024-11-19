#include "stm32f1xx_hal.h"
#include "bluetooth.h"
#include "position_sensor.h"

// Define handles
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

void SystemClock_Config(void);
void Error_Handler(void);

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIO Ports Clock
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Configure GPIO pin for Door Is Open sensor (e.g., PC13)
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Configure GPIO pin for Door Is Closed sensor (e.g., PA8)
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void MX_TIM2_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  // Initialize the RCC Oscillators according to the specified parameters
  // in the RCC_OscInitTypeDef structure.
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // Initialize the CPU, AHB and APB buses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


// Bluetooth functions
bool Bluetooth_DataAvailable()
{
  return __HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE);
}

char Bluetooth_ReadData()
{
  char data;
  HAL_UART_Receive(&huart1, (uint8_t*)&data, 1, HAL_MAX_DELAY);
  return data;
}

void Bluetooth_SendNotification(const char* message)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  while (1)
  {
    if (Bluetooth_DataAvailable())
    {
      char command = Bluetooth_ReadData();
      if (command == 'O')
        Open_Door();
      else if (command == 'C')
        Close_Door();
    }

    if (Door_IsOpen() && HAL_GetTick() % 10000 == 0)
      Bluetooth_SendNotification("Door is open!");

    HAL_Delay(100);
  }
}

void Open_Door()
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  while (!Door_IsClosed())
    HAL_Delay(100);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void Close_Door()
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  while (!Door_IsOpen())
    HAL_Delay(100);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
}

bool Door_IsOpen()
{
  return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET;
}

bool Door_IsClosed()
{
  return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET;
}
