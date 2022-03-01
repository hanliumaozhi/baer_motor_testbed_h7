/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../lan9252/lan9252.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
PROCBUFFER_OUT 	BufferOut;
PROCBUFFER_IN 	BufferIn;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
spiCTX ethercat_slave;

void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim4, 0); 
	while (__HAL_TIM_GET_COUNTER(&htim4) < us) ;
}

// load which work in velocity mode
FDCAN_TxHeaderTypeDef amber_joint_tx;
FDCAN_RxHeaderTypeDef amber_joint_rx;
uint8_t amber_joint_data[8];
uint8_t amber_joint_rx_data[8];

double load_position_;
double load_velocity_;
double load_current_;

uint32_t load_status_;

void load_power_up(FDCAN_TxHeaderTypeDef* joint_tx, uint8_t* data_buffer)
{
	joint_tx->DataLength = FDCAN_DLC_BYTES_2;
	data_buffer[0] = 0x20;
	data_buffer[1] = 0x01;
}

void load_velocity_mode(FDCAN_TxHeaderTypeDef* joint_tx, uint8_t* data_buffer)
{
	
	joint_tx->DataLength = FDCAN_DLC_BYTES_2;
	data_buffer[0] = 0x20;
	data_buffer[1] = 0x03;
}

void load_power_off(FDCAN_TxHeaderTypeDef* joint_tx, uint8_t* data_buffer)
{
	joint_tx->DataLength = FDCAN_DLC_BYTES_2;
	data_buffer[0] = 0x20;
	data_buffer[1] = 0x00;
}

union Byte2 {
	int16_t d_data;
	uint8_t buffer[2]; 
};

void load_pack_velocity_msg(FDCAN_TxHeaderTypeDef* joint_tx, uint8_t* data_buffer, double speed)
{
	joint_tx->DataLength = FDCAN_DLC_BYTES_3;
	data_buffer[0] = 0x41;
	union Byte2 byte_2;
	
	double velocity_des = speed * 57.2958;
	
	byte_2.d_data = (int16_t)velocity_des;
	
	data_buffer[1] = byte_2.buffer[1];
	data_buffer[2] = byte_2.buffer[0];
}

void load_unpack_msg(FDCAN_RxHeaderTypeDef* joint_rx, uint8_t* data_buffer)
{
	int id = joint_rx->Identifier - 0x40;
	
	if (id == 1)
	{
		int16_t p_int = (data_buffer[1] << 8) | data_buffer[2];
		int16_t v_int = (data_buffer[3] << 8) |  data_buffer[4];
		int16_t i_int =  (data_buffer[5] << 8) |  data_buffer[6];
		
		load_position_ = (p_int / 128.0f) / 57.2958f;
		load_velocity_ = (v_int * 1.0f) / 57.2958f;
		load_current_ = i_int / 256.0f; 
		
		load_status_ = (data_buffer[7] << 8) | data_buffer[0];
	}

}

// motor control func

#define P_MIN (-95.5f)
#define P_MAX (95.5f)
#define V_MIN (-45.0f)
#define V_MAX (45.0f)
#define T_MIN (-18.0f)
#define T_MAX (18.0f)
#define I_MIN (-40.0f)
#define I_MAX (40.0f)
#define KP_MIN (0.0f)     // N-m/rad
#define KP_MAX (500.0f)
#define KD_MIN (0.0f)     // N-m/rad/s
#define KD_MAX (5.0f)

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

float motor_position;
float motor_velocity;
float motor_torque;
uint32_t motor_status;

static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
	float span = x_max - x_min;
	float offset = x_min;
    
	return (uint16_t)((x - offset)*((float)((1 << bits) - 1)) / span);
}

FDCAN_TxHeaderTypeDef motor_joint_tx;
FDCAN_RxHeaderTypeDef motor_joint_rx;
uint8_t motor_joint_data[8];
uint8_t motor_joint_rx_data[8];

void motor_pack_data(FDCAN_TxHeaderTypeDef* joint_tx, float f_p, float f_v, float f_kp, float f_kd, float f_t)
{
	joint_tx->DataLength = FDCAN_DLC_BYTES_8;
	uint16_t p, v, kp, kd, t;
    
	LIMIT_MIN_MAX(f_p, P_MIN, P_MAX);
	LIMIT_MIN_MAX(f_v, V_MIN, V_MAX);
	LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
	LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
	LIMIT_MIN_MAX(f_t, T_MIN, T_MAX);
    
	p = float_to_uint(f_p, P_MIN, P_MAX, 16);            
	v = float_to_uint(f_v, V_MIN, V_MAX, 12);
	kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);
	kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);
	t = float_to_uint(f_t, T_MIN, T_MAX, 12);
    
	motor_joint_data[0] = p >> 8;
	motor_joint_data[1] = p & 0xFF;
	motor_joint_data[2] = v >> 4;
	motor_joint_data[3] = ((v & 0xF) << 4) | (kp >> 8);
	motor_joint_data[4] = kp & 0xFF;
	motor_joint_data[5] = kd >> 4;
	motor_joint_data[6] = ((kd & 0xF) << 4) | (t >> 8);
	motor_joint_data[7] = t & 0xff; 
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/// converts unsigned int to float, given range and number of bit ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span / ((float)((1 << bits) - 1)) + offset;
}

void motor_enable(FDCAN_TxHeaderTypeDef* joint_tx, uint8_t* data_buffer)
{
	joint_tx->DataLength = FDCAN_DLC_BYTES_8;
	data_buffer[0] = 0xff;
	data_buffer[1] = 0xff;
	data_buffer[2] = 0xff;
	data_buffer[3] = 0xff;
	data_buffer[4] = 0xff;
	data_buffer[5] = 0xff;
	data_buffer[6] = 0xff;
	data_buffer[7] = 0xfc;
}

void motor_zero(FDCAN_TxHeaderTypeDef* joint_tx, uint8_t* data_buffer)
{
	joint_tx->DataLength = FDCAN_DLC_BYTES_8;
	data_buffer[0] = 0xff;
	data_buffer[1] = 0xff;
	data_buffer[2] = 0xff;
	data_buffer[3] = 0xff;
	data_buffer[4] = 0xff;
	data_buffer[5] = 0xff;
	data_buffer[6] = 0xff;
	data_buffer[7] = 0xfe;
}

void motor_disable(FDCAN_TxHeaderTypeDef* joint_tx, uint8_t* data_buffer)
{
	joint_tx->DataLength = FDCAN_DLC_BYTES_8;
	data_buffer[0] = 0xff;
	data_buffer[1] = 0xff;
	data_buffer[2] = 0xff;
	data_buffer[3] = 0xff;
	data_buffer[4] = 0xff;
	data_buffer[5] = 0xff;
	data_buffer[6] = 0xff;
	data_buffer[7] = 0xfd;
}

uint64_t hs_ = 0;
float msg_time_;
void motor_unpack_msg(FDCAN_RxHeaderTypeDef* joint_rx, uint8_t* data_buffer)
{
	int id, p_int=0x00, v_int=0x00, t_int=0x00;
	id = data_buffer[0];
	p_int = (data_buffer[1] << 8)| (0x00ff & data_buffer[2]);
	v_int = (data_buffer[3] << 4) | ((0x00ff & data_buffer[4]) >> 4);
	t_int = ((data_buffer[4] & 0xF) << 8) | (0x00ff&data_buffer[5]);
	p_int &= 0x0000ffff; //16bit
	v_int &= 0x00000fff; //12bit
	t_int &= 0x00000fff; //12bit
	
	motor_position = uint_to_float(p_int,P_MIN,P_MAX,16);
	motor_velocity = uint_to_float(v_int,V_MIN,V_MAX,12);
	motor_torque = uint_to_float(t_int,I_MIN,I_MAX,12);
	//motor_status = ((data_buffer[6] << 8) | data_buffer[7]);
	
	msg_time_ = (float)hs_;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
	// 2. init load motor var
	amber_joint_tx.Identifier = 0x11;
	amber_joint_tx.IdType = FDCAN_STANDARD_ID;
	amber_joint_tx.TxFrameType = FDCAN_DATA_FRAME;
	amber_joint_tx.DataLength = FDCAN_DLC_BYTES_8;
	amber_joint_tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	amber_joint_tx.BitRateSwitch = FDCAN_BRS_OFF;
	amber_joint_tx.FDFormat = FDCAN_CLASSIC_CAN;
	amber_joint_tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	amber_joint_tx.MessageMarker = 0;
	
	//3. init motor var
	
	motor_joint_tx.Identifier = 1;
	motor_joint_tx.IdType = FDCAN_STANDARD_ID;
	motor_joint_tx.TxFrameType = FDCAN_DATA_FRAME;
	motor_joint_tx.DataLength = FDCAN_DLC_BYTES_8;
	motor_joint_tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	motor_joint_tx.BitRateSwitch = FDCAN_BRS_OFF;
	motor_joint_tx.FDFormat = FDCAN_CLASSIC_CAN;
	motor_joint_tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	motor_joint_tx.MessageMarker = 0;
	
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_Start(&hfdcan2);
	HAL_Delay(10);
	
	HAL_TIM_Base_Start(&htim2);
	
	HAL_TIM_Base_Start(&htim4);
	
	HAL_Delay(10);
	
	ethercat_slave.spi = &hspi1;
	ethercat_slave.uart = &huart2;
	ethercat_slave.bIn = &BufferIn;
	ethercat_slave.bOut = &BufferOut;
	HAL_Delay(100);
	init9252(&ethercat_slave);
	HAL_Delay(100);
	
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &amber_joint_rx, amber_joint_rx_data) == HAL_OK)
	  {
		  load_unpack_msg(&amber_joint_rx, amber_joint_rx_data);
	  }
	  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &motor_joint_rx, motor_joint_rx_data) == HAL_OK)
	  {
		  motor_unpack_msg(&motor_joint_rx, motor_joint_rx_data);
	  }
	  delay_us(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 8;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 12;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 8;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 12;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 16;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 16;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 8;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 12;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 8;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 12;
  hfdcan2.Init.DataTimeSeg2 = 2;
  hfdcan2.Init.MessageRAMOffset = 1024;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 16;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 16;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 239;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 239;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CSS_Pin */
  GPIO_InitStruct.Pin = CSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CSS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint16_t can1_error_counter = 0;
uint16_t can2_error_counter = 0;

uint32_t control_word;
int is_load_enable = 0;
int is_motor_enable = 0;
int load_init_state = 0;
int motor_init_state = 0;
uint32_t can_last_error_code = 0; 

void control()
{
	uint32_t tmp = (control_word & (2));
	int is_init = 0; 
	if ((control_word & (2)) == 2 && is_load_enable == 0)
	{
		can2_error_counter = 0;
		load_power_up(&amber_joint_tx, amber_joint_data);
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &amber_joint_tx, amber_joint_data) != HAL_OK)
		{
			can2_error_counter += 1;
		}
		is_load_enable = 1;
		load_init_state = 1;
		is_init = 1;
	}
	
	if ((control_word & 1) == 1 && is_motor_enable == 0)
	{
		can1_error_counter = 0;
		motor_enable(&motor_joint_tx, motor_joint_data); 
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &motor_joint_tx, motor_joint_data) != HAL_OK)
		{
			can1_error_counter += 1;
		}
		is_motor_enable = 1;
		motor_init_state = 1;
		is_init = 1;
	}
	if (is_init)
	{
		return; 
	}
	
	if (control_word == 0 && (is_motor_enable == 1 || is_load_enable == 1))
	{
		if (is_load_enable == 1)
		{
			load_power_off(&amber_joint_tx, amber_joint_data);
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &amber_joint_tx, amber_joint_data) != HAL_OK)
			{
				can2_error_counter += 1;
			}
		}
		if (is_motor_enable == 1)
		{
			motor_disable(&motor_joint_tx, motor_joint_data);
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &motor_joint_tx, motor_joint_data) != HAL_OK)
			{
				can1_error_counter += 1;
			}
		}
		is_motor_enable = 0;
		is_load_enable = 0;
	}
	
	if (load_init_state == 1)
	{
		load_velocity_mode(&amber_joint_tx, amber_joint_data);
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &amber_joint_tx, amber_joint_data) != HAL_OK)
		{
			can2_error_counter += 1;
		}
		load_init_state = 0;
	}
	
	if (motor_init_state == 1)
	{
		motor_zero(&motor_joint_tx, motor_joint_data);
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &motor_joint_tx, motor_joint_data) != HAL_OK)
		{
			can1_error_counter += 1;
		}
		motor_init_state = 0;
	}
	
	if (is_load_enable == 1)
	{
		load_pack_velocity_msg(&amber_joint_tx, amber_joint_data, BufferOut.Cust.load_velocity);
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &amber_joint_tx, amber_joint_data) != HAL_OK)
		{
			can2_error_counter += 1;
		}
	}
	
	if (is_motor_enable == 1)
	{
		motor_pack_data(&motor_joint_tx, BufferOut.Cust.motor_position_des, BufferOut.Cust.motor_velocity_des,
			BufferOut.Cust.motor_kp, BufferOut.Cust.motor_kd, BufferOut.Cust.motor_torque_des);
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &motor_joint_tx, motor_joint_data) != HAL_OK)
		{
			can1_error_counter += 1;
		}
	}
	
}

void pack_ethercat_data()
{
	BufferIn.Cust.hs = hs_;
	BufferIn.Cust.load_velocity = load_velocity_;
	BufferIn.Cust.load_status = load_status_;
	BufferIn.Cust.can1_error_counter = can_last_error_code;
	BufferIn.Cust.can2_error_counter = (can1_error_counter<< 16) | (can2_error_counter & 0xffff);
	
	// motor
	BufferIn.Cust.motor_position_act = motor_position;
	BufferIn.Cust.motor_velocity_act = motor_velocity;
	BufferIn.Cust.motor_torque_act = motor_torque;
	BufferIn.Cust.motor_status = motor_status;
	
	// we use load_current to
	BufferIn.Cust.load_current = msg_time_;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM5)
	{
		pack_ethercat_data();
		main_task(&ethercat_slave);
		uint64_t tmp_hs_ = BufferOut.Cust.hs;
		if (tmp_hs_ > hs_ || tmp_hs_ == 1)
		{
			control_word = BufferOut.Cust.control_word;
			
			control();
			
			hs_ = tmp_hs_;
			
			// pack data
			// error check;
			FDCAN_ErrorCountersTypeDef ErrorCounters;
			uint8_t error_counter1;
			uint8_t error_counter2;
			uint8_t error_counter3;
			uint8_t error_counter4;
			HAL_FDCAN_GetErrorCounters(&hfdcan1, &ErrorCounters);
			error_counter1 = (uint8_t)ErrorCounters.RxErrorCnt;
			error_counter2 = (uint8_t)ErrorCounters.TxErrorCnt;
			HAL_FDCAN_GetErrorCounters(&hfdcan2, &ErrorCounters);
			error_counter3 = (uint8_t)ErrorCounters.RxErrorCnt;
			error_counter4 = (uint8_t)ErrorCounters.TxErrorCnt;
			
			
			uint32_t can1_lec = READ_REG(hfdcan1.Instance->PSR);
			uint32_t can2_lec = READ_REG(hfdcan2.Instance->PSR);
				
			can1_lec = can1_lec & 0x0007;
			can2_lec = can2_lec & 0x0007;

			can_last_error_code = (can2_lec << 8) | can1_lec;
				
			if (hs_ > 100)
			{
				if (error_counter1 > 0 || error_counter2 > 0)
				{
					can1_error_counter += 1;
					
				}
				if (error_counter3 > 0 || error_counter4 > 0) 
				{
					can2_error_counter += 1;
				}
			}
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

