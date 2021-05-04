/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define REC_WAIT_START_US    75
#define REC_WAIT_PARAMS_US   (SERVO_MAX_PARAMS * 5)
#define REC_WAIT_MAX_RETRIES 200
#define SERVO_MAX_PARAMS (REC_BUFFER_LEN - 5)

volatile uint8_t receiveBuffer[REC_BUFFER_LEN];
volatile uint8_t* volatile receiveBufferStart = receiveBuffer;
volatile uint8_t* volatile receiveBufferEnd = receiveBuffer;



/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint8_t Buffer[10];
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
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
size_t getServoBytesAvailable (void);
uint8_t getServoByte (void);
uint8_t getServoAngle ( uint8_t ID,  uint16_t* angleValue);
uint8_t getServoResponse (void);
uint8_t setServoAngle( uint8_t ID,uint8_t angle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum ServoCommand
{
    PING = 1,
    READ = 2,
    WRITE = 3
} ServoCommand;



typedef struct ServoResponse
{
    uint8_t id;
    uint8_t length;
    uint8_t error;
    uint8_t params[SERVO_MAX_PARAMS];
    uint8_t checksum;
} ServoResponse;

ServoResponse response;
uint8_t servoErrorCode = 0;
/* USER CODE END 0 */


int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t angle=512;
	uint16_t delta=10;
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  Buffer[0]='U';

  /* USER CODE END 2 */
 // HAL_StatusTypeDef
//  HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
//  HAL_UART_Transmit(&huart3, Buffer, 1, 100);

//  HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  setAngleLimit(4, 0, 1023);
//  setServoAngle(4, angle);
  reset(4);
  //turn(4, RIGHT,80);
  while (1)
  {
    /* USER CODE END WHILE */
   if (angle>700)
   {
	   angle=700;
	 delta=-5;
   }
   if (angle<200){
	   angle=200;
	   delta=+5;
   }
    /* USER CODE BEGIN 3 */
	  HAL_Delay (20); /* Insert delay 100 ms */
	 // getServoAngle(4, &angle);
	move(2, angle);
	angle+=delta;
	//  setAngleLimit(4, 0, 1023);
	//  readPosition(4);

	// turn(4, RIGHT,00);


  }
  /* USER CODE END 3 */

}

void clearServoReceiveBuffer (void)
{
    receiveBufferStart = receiveBufferEnd;
}

uint8_t getAndCheckResponse (const uint8_t servoId)
{
    if (!getServoResponse())
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Servo %d did not respond correctly or at all\n", (int)servoId);
        #endif
        return 0;
    }

    if (response.id != servoId)
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Response ID %d does not match command ID %d\n", (int)response.id);
        #endif
        return 0;
    }

    if (response.error != 0)
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Response error code was nonzero (%d)\n", (int)response.error);
        #endif
        return 0;
    }

    return 1;
}


uint8_t getServoResponse (void)
{
    uint8_t retries = 0;

    clearServoReceiveBuffer();

    while (getServoBytesAvailable() < 4)
    {
        retries++;
        if (retries > REC_WAIT_MAX_RETRIES)
        {
            #ifdef SERVO_DEBUG
            printf ("Too many retries at start\n");
            #endif
            return 0;
        }

     //   mWaitus (REC_WAIT_START_US);
    }
    retries = 0;

    getServoByte();  // servo header (two 0xff bytes)
    getServoByte();

    response.id = getServoByte();
    response.length = getServoByte();

    if (response.length > SERVO_MAX_PARAMS)
    {
        #ifdef SERVO_DEBUG
        printf ("Response length too big: %d\n", (int)response.length);
        #endif
        return 0;
    }

    while (getServoBytesAvailable() < response.length)
    {
        retries++;
        if (retries > REC_WAIT_MAX_RETRIES)
        {
            #ifdef SERVO_DEBUG
            printf ("Too many retries waiting for params, got %d of %d params\n", getServoBytesAvailable(), response.length);
            #endif
            return 0;
        }

     //   mWaitus (REC_WAIT_PARAMS_US);
    }

    response.error = getServoByte();
    servoErrorCode = response.error;

    for (uint8_t i = 0; i < response.length - 2; i++)
        response.params[i] = getServoByte();


    uint8_t calcChecksum = response.id + response.length + response.error;
    for (uint8_t i = 0; i < response.length - 2; i++)
        calcChecksum += response.params[i];
    calcChecksum = ~calcChecksum;

    const uint8_t recChecksum = getServoByte();
    if (calcChecksum != recChecksum)
    {
        #ifdef SERVO_DEBUG
        printf ("Checksum mismatch: %x calculated, %x received\n", calcChecksum, recChecksum);
        #endif
        return 0;
    }

    return 1;
}

/*--------------------------------------*/

int setAngleLimit(uint8_t ID, uint16_t CWLimit, uint16_t CCWLimit)
{
  const uint8_t length = 8;
		uint8_t packet[length];
		uint8_t Checksum;
    //  uint8_t packet[9];

		uint8_t pdata[8];
		HAL_StatusTypeDef val;

	uint8_t CW_H,CW_L,CCW_H,CCW_L;
    CW_H = CWLimit >> 8;
    CW_L = CWLimit;                // 16 bits - 2 x 8 bits variables
    CCW_H = CCWLimit >> 8;
    CCW_L = CCWLimit;



	Checksum = (~(ID + AX_CCW_CW_LENGTH + AX_WRITE_DATA + AX_CW_ANGLE_LIMIT_L + CW_H + CW_L + AX_CCW_ANGLE_LIMIT_L + CCW_H + CCW_L)) & 0xFF;

	packet[0] = AX_START;
	packet[1] = AX_START;
	packet[2] = ID;
	packet[3] = AX_CCW_CW_LENGTH;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_CW_ANGLE_LIMIT_L;
	packet[6] = CW_L;
	packet[7] = CW_H;
	packet[8] = AX_CCW_ANGLE_LIMIT_L;
	packet[9] = CCW_L;
	packet[10] = CCW_H;
	packet[11] = Checksum;

	val=HAL_UART_Receive(&huart3, pdata, 2, 100);
	    //return (sendAXPacket(packet, length));
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);//passer en mode transmission
	    HAL_UART_Transmit(&huart3, packet, 12, 100);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	    val=HAL_UART_Receive(&huart3, pdata, 8, 100);
	    return 1;
}





uint8_t getServoByte (void)
{
    receiveBufferStart++;
    if (receiveBufferStart >= receiveBuffer + REC_BUFFER_LEN)
        receiveBufferStart = receiveBuffer;

    return *receiveBufferStart;

}

size_t getServoBytesAvailable (void)
{
    volatile uint8_t *start = receiveBufferStart;
    volatile uint8_t *end = receiveBufferEnd;

    if (end >= start)
        return (size_t)(end - start);
    else
        return (size_t)(REC_BUFFER_LEN - (start - end));
}



/**
  * @brief System Clock Configuration
  * @retval None
  */

/*---------------------------------------------------------*/
void turn(uint8_t ID, uint8_t SIDE, uint16_t Speed)
{
	uint8_t Checksum;
	uint8_t packet[9];
	HAL_StatusTypeDef val;
	uint8_t pdata[8];
		if (SIDE == LEFT)
		{
			int Speed_H,Speed_L;
			Speed_H = Speed >> 8;
			Speed_L = Speed;                     // 16 bits - 2 x 8 bits variables

			//const unsigned int length = 9;
			//unsigned char packet[length];

			Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + Speed_L + Speed_H)) & 0xFF;

		    packet[0] = AX_START;
		    packet[1] = AX_START;
		    packet[2] = ID;
		    packet[3] = AX_SPEED_LENGTH;
		    packet[4] = AX_WRITE_DATA;
		    packet[5] = AX_GOAL_SPEED_L;
		    packet[6] = Speed_L;
		    packet[7] = Speed_H;
		    packet[8] = Checksum;

		    //return (sendAXPacket(packet, length));
		}

		else
		{
			char Speed_H,Speed_L;
			Speed_H = (Speed >> 8) + 4;
			Speed_L = Speed;                     // 16 bits - 2 x 8 bits variables

		//	const unsigned int length = 9;


			Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + Speed_L + Speed_H)) & 0xFF;

		    packet[0] = AX_START;
		    packet[1] = AX_START;
		    packet[2] = ID;
		    packet[3] = AX_SPEED_LENGTH;
		    packet[4] = AX_WRITE_DATA;
		    packet[5] = AX_GOAL_SPEED_L;
		    packet[6] = Speed_L;
		    packet[7] = Speed_H;
		    packet[8] = Checksum;

		    //return (sendAXPacket(packet, length));
		}
		val=HAL_UART_Receive(&huart3, pdata, 2, 100);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);//passer en mode transmission
		  HAL_UART_Transmit(&huart3, packet, 9, 100);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); //passer en mode réception
		  val=HAL_UART_Receive(&huart3, pdata, 6, 100);

}


int readPosition(uint8_t ID)
{
	const uint8_t length = 8;
	uint8_t packet[length];
	uint8_t Checksum;
//	uint8_t packet[9];
    uint8_t Position_H,Position_L;
	uint8_t pdata[8];
	HAL_StatusTypeDef val;

    Checksum = (~(ID + AX_POS_LENGTH + AX_READ_DATA + AX_PRESENT_POSITION_L + AX_BYTE_READ_POS)) & 0xFF;

	packet[0] = AX_START;
	packet[1] = AX_START;
	packet[2] = ID;
	packet[3] = AX_POS_LENGTH;
	packet[4] = AX_READ_DATA;
	packet[5] = AX_PRESENT_POSITION_L;
	packet[6] = AX_BYTE_READ_POS;
	packet[7] = Checksum;

    val=HAL_UART_Receive(&huart3, pdata, 2, 100);
    //return (sendAXPacket(packet, length));
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);//passer en mode transmission
    HAL_UART_Transmit(&huart3, packet, 8, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    val=HAL_UART_Receive(&huart3, pdata, 8, 100);
    return 1;
}



/*----------------------------------------------------------------*/

void move(uint8_t ID, uint16_t Position)
{
	uint8_t Checksum;
	uint8_t packet[9];
    uint8_t Position_H,Position_L;
	uint8_t pdata[8];
	HAL_StatusTypeDef val;

    Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
    Position_L = Position;
    //unsigned char packet[length];

	Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H)) & 0xFF;

    packet[0] = AX_START;
    packet[1] = AX_START;
    packet[2] = ID;
    packet[3] = AX_GOAL_LENGTH;
    packet[4] = AX_WRITE_DATA;
    packet[5] = AX_GOAL_POSITION_L;
    packet[6] = Position_L;
    packet[7] = Position_H;
    packet[8] = Checksum;


    val=HAL_UART_Receive(&huart3, pdata, 2, 100);
    //return (sendAXPacket(packet, length));
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);//passer en mode transmission
    HAL_UART_Transmit(&huart3, packet, 9, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    val=HAL_UART_Receive(&huart3, pdata, 6, 100);
    return ;
}

/*----------------------------------------------------------------*/

int reset(uint8_t ID)
{
	uint8_t Checksum;
		HAL_StatusTypeDef val;
		uint8_t packet[11];
	    uint8_t Position_H,Position_L,Speed_H,Speed_L;
		uint8_t pdata[6];

	Checksum = (~(ID + AX_RESET_LENGTH + AX_RESET)) & 0xFF;

	packet[0] = AX_START;
	packet[1] = AX_START;
	packet[2] = ID;
	packet[3] = AX_RESET_LENGTH;
	packet[4] = AX_RESET;
	packet[5] = Checksum;

	val=HAL_UART_Receive(&huart3, pdata, 2, 100);
	    //return (sendAXPacket(packet, length));
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);//passer en mode transmission
	    HAL_UART_Transmit(&huart3, packet, 8, 100);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	    val=HAL_UART_Receive(&huart3, pdata, 8, 100);
	    return 1;
}


/*--------------------------------*/

void moveSpeed(uint8_t ID, int Position, int Speed)
{
	uint8_t Checksum;
	HAL_StatusTypeDef val;
	uint8_t packet[11];
    char Position_H,Position_L,Speed_H,Speed_L;
	uint8_t pdata[8];

    Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
    Position_L = Position;
    Speed_H = Speed >> 8;           // 16 bits - 2 x 8 bits variables
    Speed_L = Speed;


    //unsigned char packet[length];

	Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H)) & 0xFF;

    packet[0] = AX_START;
    packet[1] = AX_START;
    packet[2] = ID;
    packet[3] = AX_GOAL_LENGTH;
    packet[4] = AX_WRITE_DATA;
    packet[5] = AX_GOAL_POSITION_L;
    packet[6] = Position_L;
    packet[7] = Position_H;
    packet[8] = Speed_L;
    packet[9] = Speed_H;
    packet[10] = Checksum;

    //return (sendAXPacket(packet, length));
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);//passer en mode transmission
    HAL_UART_Transmit(&huart3, packet, 9, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    val=HAL_UART_Receive(&huart3, pdata, 6, 100);
}

void changeBaudRate(){
	uint8_t Checksum;
	HAL_StatusTypeDef val;
	uint8_t pdata[8];
//	Checksum = (~(ID +  AX_ACTION + GOAL_ANGLE + lowByte + highByte)) & 0xFF;

  /*  uint8_t params[8] = {AX_START,AX_START,ID,AX_ACTION,GOAL_ANGLE,
                               lowByte,
                               highByte,
							   Checksum};*/
	uint8_t params[11]={0xFF,0xFF,4,7,3,0x1E,0,2,0,2,0xCF};
//	uint8_t params[11]={0xFF,0xFF,4,7,3,0x1A,1,1,0x40,0x40,0x55};
    val=HAL_UART_Receive(&huart3, pdata, 2, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);//passer en mode transmission
    HAL_UART_Transmit(&huart3, params, 11, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
 //   HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
    val=HAL_UART_Receive(&huart3, pdata, 6, 100);

    return 1;
}

/*----------------------------------------------*/
uint8_t setServoAngle( uint8_t ID,uint8_t angle)
{
	uint8_t Checksum;
	HAL_StatusTypeDef val;
	uint8_t pdata[8];

 //   if (angle < 0 || angle > 300)
 //       return 0;

    // angle values go from 0 to 0x3ff (1023)
    const uint16_t angleValue = (uint16_t)(angle * (1023.0 / 300.0));

    const uint8_t highByte = (uint8_t)((angleValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(angleValue & 0xff);
	Checksum = (~(ID +  AX_WRITE_DATA +4+ GOAL_ANGLE + lowByte + highByte));

   uint8_t params[9] = {AX_START,AX_START,ID,AX_WRITE_DATA,4,GOAL_ANGLE,
                               lowByte,
                               highByte,
							   Checksum};
//   uint8_t params[9]={0xFF,0xFF,4,5,4,0x1E,0xFF,3,0xD2};
//   >[Dynamixel]:FF FF 01 05 04 1E FF 03 D5 (LEN:009)
//	uint8_t params[11]={0xFF,0xFF,4,7,3,e,0,2,0xCF}; //rotation
//	uint8_t params[11]={0xFF,0xFF,4,7,3,0x1A,1,1,0x40,0x40,0x55};
    val=HAL_UART_Receive(&huart3, pdata, 4, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);//passer en mode transmission
    HAL_UART_Transmit(&huart3, params, 9, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
 //   HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
    val=HAL_UART_Receive(&huart3, pdata, 6, 100);

    return 1;
}
/*---------------------------------*/
uint8_t getServoAngle ( uint8_t ID,  uint16_t* angleValue)
{
	uint8_t retour;
    const uint8_t params[2] = {CURRENT_ANGLE,
                               4};  // read two bytes, starting at address CURRENT_ANGLE

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);//passer en mode transmission
    HAL_UART_Transmit(&huart3, params, 2, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

    if (!getAndCheckResponse (ID))
    	retour=0;
         return retour;

     //uint16_t
         retour=1;
    *angleValue = response.params[1];
     *angleValue <<= 8;
     *angleValue |= response.params[0];

 //    *angle = (float)angleValue * 300.0 / 1023.0;

/*
    uint16_t angleValue = response.params[1];
    angleValue <<= 8;
    angleValue |= response.params[0];

    *angle = (float)angleValue * 300.0 / 1023.0;
*/
    return retour;
}
/*-------------------------------------------------------------------------------*/
/*void sendAXPacket(uint8_t * packet, unsigned int length)
{
	switchCom(Direction_Pin, TX_MODE); 	// Switch to Transmission  Mode

	sendData(packet, length);			// Send data through sending buffer
	flush(); 							// Wait until buffer is empty

	switchCom(Direction_Pin, RX_MODE); 	// Switch back to Reception Mode

	return (read_error());              // Return the read error
}
*/
/*--------------------------------------------------------------*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 912000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
//  USART3->CR1|=1<<5;
//  HAL_UART_MspInit(&huart3);
  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS_EN_Pin SMPS_V1_Pin SMPS_SW_Pin */
  GPIO_InitStruct.Pin = SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMPS_PG_Pin */
  GPIO_InitStruct.Pin = SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 LD4_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
