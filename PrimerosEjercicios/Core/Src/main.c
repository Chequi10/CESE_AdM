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
#include "string.h"
#include<stdint.h>
#include<stdio.h>

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
#include "asm_func.h"
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void zeros (uint32_t * vector, uint32_t longitud);
void productoEscalar32 (uint32_t * vectorIn, uint32_t * vectorOut, uint32_t longitud, uint32_t escalar);
void productoEscalar16 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);
void productoEscalar12 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);

void filtroVentana10 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitudVectorIn);
void pack32to16 (int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);
int32_t max (int32_t * vectorIn, uint32_t longitud);
void downsampleM (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N);
void invertir (uint16_t * vector, uint32_t longitud);



uint32_t vector[4];
uint32_t longitud;

uint32_t vectorIn[]={0,1,2,3};
uint32_t vectorOut32[4];


uint16_t vectorIn16[]={0,1,2,3};
uint16_t vectorOut16[4];

uint16_t vectorOut12[4];

uint16_t vector2In16[]={0,1,2,3,4,5,6,7,8,9};
uint16_t vector2Out16[10]={0};

int32_t vectorIn32Int[]={0,1,2,3};
int32_t vectorOut32Int[4]={0};
int16_t vectorOut16Int[4]={0};

uint32_t vectorInMax[]={0,1,2,30,4,5,60,7,8,9};

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void PrivilegiosSVC (void)
{
    // Obtiene valor del registro de 32 bits del procesador llamado "control".
    // El registro guarda los siguientes estados:
    // bit 2: Uso de FPU en el contexto actual. Usado=1, no usado=0.
    // bit 1: Mapeo del stack pointer(sp). MSP=0, PSP=1.
    // bit 0: Modo de ejecucion en Thread. Privilegiado=0, No privilegiado=1.
    //        Recordar que este valor solo se usa en modo Thread. Las
    //        interrupciones siempre se ejecutan en modo Handler con total
    //        privilegio.
    uint32_t x = __get_CONTROL ();

    // Actividad de debug: Ver registro "control" y valor de variable "x".
    //__BKPT (0);

    x |= 1;
    // bit 0 a modo No privilegiado.
    __set_CONTROL (x);

    // En este punto se estaria ejecutando en modo No privilegiado.
    // Lectura del registro "control" para confirmar.
    x = __get_CONTROL ();

    // Actividad de debug: Ver registro "control" y valor de variable "x".
    //__BKPT (0);

    x &= ~1u;
    // Se intenta volver a modo Privilegiado (bit 0, valor 0).
    __set_CONTROL (x);

    // Confirma que esta operacion es ignorada por estar ejecutandose en modo
    // Thread no privilegiado.
    x = __get_CONTROL ();

    // Actividad de debug: Ver registro "control" y valor de variable "x".
    //__BKPT (0);

    // En este punto, ejecutando en modo Thread no privilegiado, la unica forma
    // de volver a modo privilegiado o de realizar cualquier cambio que requiera
    // modo privilegiado, es pidiendo ese servicio a un hipotetico sistema
    // opertivo de tiempo real.
    // Para esto se invoca por software a la interrupcion SVC (Supervisor Call)
    // utilizando la instruccion "svc".
    // No hay intrinsics para realizar esta tarea. Para utilizar la instruccion
    // es necesario implementar una funcion en assembler. Ver el archivo
    // asm_func.S.
    asm_svc ();

    // El sistema operativo (el handler de SVC) deberia haber devuelto el modo
    // de ejecucion de Thread a privilegiado (bit 0 en valor 0).
    x = __get_CONTROL ();

    // Fin del ejemplo de SVC
}


void zeros (uint32_t * vector, uint32_t longitud)
{
	  for(uint32_t i=0; i<longitud; i++)
	   {
		  vector[i]=0;

	   }

}

void productoEscalar32 (uint32_t * vectorIn, uint32_t * vectorOut, uint32_t longitud, uint32_t escalar)

{
	  for(uint32_t i=0; i<longitud; i++)
	   {
		  vectorOut[i]=vectorIn[i]*escalar;

	   }

}

void productoEscalar16 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar)
{
	  for(uint32_t i=0; i<longitud; i++)
	   {
		  vectorOut[i]=vectorIn[i]*escalar;

	   }

}

void productoEscalar12 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar)
{

	for(uint32_t i=0 ; i < longitud ; i++)

	{
		vectorOut[i] = vectorIn[i]*escalar;

		     if(vectorOut[i] >= 4095)
	         	{
	        		vectorOut[i] = 4095;
	        	}

	}

}



void filtroVentana10 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitudVectorIn)
{

	uint8_t ancho = 10;
	uint32_t i=0;
	uint32_t a=0;

		for(i = 0; (i + ancho) <= longitudVectorIn; i++)
		{
			for(a = i; a < (ancho + i); a++)
			{
				vectorOut[i] += vectorIn[a];
			}
			vectorOut[i] /= ancho;
		}
}


void pack32to16 (int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud)
{
	uint32_t a;

	for(a=0 ; a<longitud ; a++)
		{
			vectorOut[a] = vectorIn[a]>>16;

		}

}


int32_t max (int32_t * vectorIn, uint32_t longitud)
{

	int32_t posicion = 0;
	uint32_t a;

	for(a=0 ; a<longitud ; a++)
	{

		if(vectorIn[a] > vectorIn[posicion])
			{
				posicion = a;
			}

	}
	return posicion;

}


void downsampleM(int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N)
{
	uint32_t Temporal = 0;
	uint32_t indice = 0;
	uint32_t a=0;

	for(a = 0; a < longitud; a++){

		if(Temporal != N)
			{
				vectorOut[indice] = vectorIn[a];
				indice++;
				Temporal++;
			}
			else
			{
				vectorIn[a] = vectorIn[a];
				Temporal = 0;
			}
	}
}



void invertir (uint16_t * vector, uint32_t longitud)
{
	uint16_t temporal=0;
	uint32_t posicionleft = 0;
	uint32_t posicionright = longitud-1;

	while(posicionleft < posicionright)
		{
			temporal=vector[posicionleft];
			vector[posicionleft]=vector[posicionright];
			vector[posicionright]=temporal;
			posicionleft++;
			posicionright--;
		}

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
  /* USER CODE BEGIN 2 */
  PrivilegiosSVC ();

// const uint32_t Resultado = asm_sum (5, 3);
         //  DWT->CTRL |=1 << DWT_CTRL_CYCCNTENA_Pos;
//  asm_zeros(vector,4);
//  asm_productoEscalar32 (vectorIn,vectorOut32,4,4);
//  asm_productoEscalar16 (vectorIn16,vectorOut16,4,5);
//  asm_productoEscalar12 (vectorIn16,vectorOut12,4,1400);
//  asm_invertir(vector2In16, 10);
//  asm_max (vectorInMax, 10);
  asm_pack32to16 (vectorIn32Int, vectorOut16Int, 4);
//  asm_filtroVentana10 (vector2In16, vector2Out16, 10);
//  asm_downsampleM (vectorIn32Int,vectorOut32Int,4, 2);


         //   DWT->CYCCNT =0;
 // zeros (vector,4);
         //  uint32_t volatile c=DWT->CYCCNT;
// productoEscalar32 (vectorIn,vectorOut32,4,3);
// productoEscalar16 (vectorIn16,vectorOut16,4,5);
// productoEscalar12 (vectorIn16,vectorOut12,4,1400);
// invertir(vector2In16, 10);
// int32_t vec = max (vectorInMax, 9);
// pack32to16 (vectorIn32Int, vectorOut16Int, 4);
// filtroVentana10 (vector2In16, vector2Out16, 10);
// downsampleM (vectorIn32Int,vectorOut32Int,4, 2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
