/**
 * SSTV Transmission Example using SX1276 LoRa module in FSK Continuous mode
 * Platform: STM32F103C8 (Blue Pill)
 * Framework: STM32Cube HAL
 *
 * This example uses the DIO2 pin of SX1276 for audio modulation (AFSK).
 * A PWM signal is generated on PB10 and connected to DIO2.
 * The module is set to FSK Continuous mode where DIO2 acts as DATA input.
 *
 * Pin connections:
 *   SX1276      Blue Pill
 *   ------      ---------
 *   VCC         3.3V
 *   GND         GND
 *   SCK         PA5 (SPI1_SCK)
 *   MISO        PA6 (SPI1_MISO)
 *   MOSI        PA7 (SPI1_MOSI)
 *   NSS         PA4
 *   RST         PB0
 *   DIO0        PA1
 *   DIO2        PB10 (tone output from STM32, TIM2_CH3)
 *
 * SSTV mode: Robot 72
 */

#include "stm32f1xx_hal.h"
#include <LoRa.h>
#include "FATFS/App/fatfs.h"
#include "FATFS_SD.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define SSTV_MINI_IMPLEMENTATION
#include "sstv_mini.h"

/* Pin definitions */
#define LORA_NSS_PIN        GPIO_PIN_4
#define LORA_NSS_PORT       GPIOA
#define LORA_RST_PIN        GPIO_PIN_0
#define LORA_RST_PORT       GPIOB
#define LORA_DIO0_PIN       GPIO_PIN_1
#define LORA_DIO0_PORT      GPIOA

/* Tone output pin (connected to DIO2 of SX1276) */
#define TONE_PIN            GPIO_PIN_10
#define TONE_PORT           GPIOB

/* LED indicator pin (PC13) for error blinking */
#define LED_PIN             GPIO_PIN_13
#define LED_PORT            GPIOC

/* Global variables */
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;     /* For tone generation (PWM on PB10) */
TIM_HandleTypeDef htim3;     /* For microsecond timing */
LoRa myLoRa;
sstv_ctx_t sstv;

/////////////////////////////////for FATFS///////////////
extern SPI_HandleTypeDef   hspi2;
#define HSPI_SDCARD        &hspi2
#define SD_CS_PORT         GPIOB
#define SD_CS_PIN          GPIO_PIN_12
#define SPI_TIMEOUT        100
SPI_HandleTypeDef hspi2;
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);

static void SD_Card_Test(void)
{
  FATFS FatFs;
  FIL Fil;
  FRESULT FR_Status;
//   FATFS *FS_Ptr;
  UINT WWC; // Read/Write Word Counter
//   DWORD FreeClusters;
//   uint32_t TotalSize, FreeSpace;
  char RW_Buffer[200];
  do
  {
    //------------------[ Mount The SD Card ]--------------------
	    FR_Status = f_mount(&FatFs, "", 1);
	    if (FR_Status != FR_OK)
	    {
	    //  sprintf(TxBuffer, "Error! While Mounting SD Card, Error Code: (%i)\r\n", FR_Status);
	    //  UART_Print(TxBuffer);
	      break;
	    }
        //------------------[ Open A Text File For Write & Write Data ]--------------------
    //Open the file
    FR_Status = f_open(&Fil, "1.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    if(FR_Status != FR_OK)
    {
     // sprintf(TxBuffer, "Error! While Creating/Opening A New Text File, Error Code: (%i)\r\n", FR_Status);
    //  UART_Print(TxBuffer);
      break;
    }
    //sprintf(TxBuffer, "Text File Created & Opened! Writing Data To The Text File..\r\n\n");
    //UART_Print(TxBuffer);
    // (1) Write Data To The Text File [ Using f_puts() Function ]
    f_puts("it works. puts()\n", &Fil);
    // (2) Write Data To The Text File [ Using f_write() Function ]
    strcpy(RW_Buffer, "it works. f_write()\r\n");
    f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
    // Close The File
    f_close(&Fil);
    }
   while(0);  
}
/////////////////////////////////////////////////////////

/* Current tone state */
static volatile uint8_t tone_active = 0;

/* Test image - single 320px line with color stripes (8 colors, 40 pixels each) */
static uint32_t test_line[320] = {
    /* Black (40 pixels) */
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,

    /* Blue (40 pixels) */
    0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF,
    0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF,
    0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF,
    0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF, 0x0000FF,

    /* Green (40 pixels) */
    0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00,
    0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00,
    0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00,
    0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00,

    /* Cyan (40 pixels) */
    0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF,
    0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF,
    0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF,
    0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF,

    /* Red (40 pixels) */
    0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000,
    0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000,
    0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000,
    0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000, 0xFF0000,

    /* Magenta (40 pixels) */
    0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF,
    0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF,
    0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF,
    0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF,

    /* Yellow (40 pixels) */
    0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00,
    0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00,
    0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00,
    0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00,

    /* White (40 pixels) */
    0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF,
    0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF,
    0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF,
    0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF
};

/* Function prototypes */
static void SystemClock_Config(void);
static void GPIO_Init(void);
static void SPI1_Init(void);
static void TIM2_Init(void);
static void TIM3_Init(void);
static void Error_Handler(void);

/* Tone functions (Arduino-like) */
static void tone(uint16_t freq);
static void noTone(void);

/* SSTV callbacks */
static void sstv_tone_callback(uint16_t freq);
static void sstv_no_tone_callback(void);
static void sstv_delay_us_callback(uint32_t us);
static uint32_t sstv_micros_callback(void);

/* Delay in microseconds */
static void delay_us(uint32_t us);

/* Get microseconds counter */
static uint32_t micros(void);

int main(void)
{
    uint16_t state;

    /* HAL initialization */
    HAL_Init();

    /* Configure system clock to 72 MHz */
    SystemClock_Config();

    /* Initialize peripherals */
    GPIO_Init();
    SPI1_Init();
    TIM2_Init();  /* For tone PWM */
    TIM3_Init();  /* For microsecond counter */

//////////////////////////For FATFS///////////////////////////////////
  MX_GPIO_Init();
  MX_SPI2_Init();
//   MX_USART1_UART_Init();
  MX_FATFS_Init();
  
   SD_Card_Test();
//////////////////////////////////////////////////////////////////////

    /* Start microsecond timer */
    HAL_TIM_Base_Start(&htim3);

    /* Initialize LoRa module */
    myLoRa = newLoRa();

    myLoRa.hSPIx       = &hspi1;
    myLoRa.CS_port     = LORA_NSS_PORT;
    myLoRa.CS_pin      = LORA_NSS_PIN;
    myLoRa.reset_port  = LORA_RST_PORT;
    myLoRa.reset_pin   = LORA_RST_PIN;
    myLoRa.DIO0_port   = LORA_DIO0_PORT;
    myLoRa.DIO0_pin    = LORA_DIO0_PIN;

    myLoRa.frequency   = 433;           /* 433 MHz base frequency */
    myLoRa.power       = POWER_20db;    /* Maximum power */
    myLoRa.overCurrentProtection = 120; /* 120 mA */

    /* Reset the module */
    LoRa_reset(&myLoRa);

    /* Initialize in FSK mode */
    state = LoRa_beginFSK(&myLoRa);
    if (state != LORA_OK) {
        Error_Handler();
    }

    /* Start direct/continuous mode (DIO2 = DATA input) */
    LoRa_startDirectMode(&myLoRa);

    /* Initialize SSTV with Robot 72 mode */
    state = sstv_init(&sstv,
                      &SSTV_MODE_ROBOT_72,
                      sstv_tone_callback,
                      sstv_no_tone_callback,
                      sstv_delay_us_callback,
                      sstv_micros_callback);
    if (state != SSTV_ERR_NONE) {
        Error_Handler();
    }

    /* Set timing correction factor for STM32F103 @ 72MHz */
    sstv_set_correction(&sstv, 0.92f);

    /* Main loop - transmit test image every 5 seconds */
    while (1)
    {
        /* Start transmitter */
        LoRa_transmitDirect(&myLoRa);

        /* Send SSTV header */
        sstv_send_header(&sstv);

        /* Send all image lines */
        uint16_t height = sstv_get_picture_height(&sstv);
        for (uint16_t i = 0; i < height; i++) {
            sstv_send_line(&sstv, test_line);
        }

        /* Stop tone and transmitter */
        sstv_no_tone(&sstv);
        LoRa_standby(&myLoRa);

        /* Wait before next transmission */
        HAL_Delay(5000);
    }
}

/**
 * Generate a tone on PB10 (TIM2_CH3) at specified frequency
 * Similar to Arduino tone() function
 */
static void tone(uint16_t freq)
{
    if (freq == 0) {
        noTone();
        return;
    }

    /* Calculate timer period for desired frequency
     * Timer clock = 72 MHz (APB1 timer clock with prescaler = 0)
     * Period = Timer_clock / freq - 1
     * For audio frequencies (1000-3000 Hz), we use prescaler = 71
     * So timer clock = 1 MHz, and period = 1000000 / freq - 1 */
    uint32_t period = (1000000 / freq) - 1;
    if (period < 1) period = 1;
    if (period > 65535) period = 65535;

    /* Update timer period */
    __HAL_TIM_SET_AUTORELOAD(&htim2, period);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, period / 2);  /* 50% duty cycle */

    /* Start PWM if not already running */
    if (!tone_active) {
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
        tone_active = 1;
    }
}

/**
 * Stop tone generation
 * Similar to Arduino noTone() function
 */
static void noTone(void)
{
    if (tone_active) {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
        tone_active = 0;
    }
    /* Ensure pin is low when not generating tone */
    HAL_GPIO_WritePin(TONE_PORT, TONE_PIN, GPIO_PIN_RESET);
}

/**
 * SSTV tone callback - generates tone and starts TX
 */
static void sstv_tone_callback(uint16_t freq)
{
    tone(freq);
}

/**
 * SSTV no-tone callback - stops tone
 */
static void sstv_no_tone_callback(void)
{
    noTone();
}

/**
 * SSTV delay callback - microsecond delay
 */
static void sstv_delay_us_callback(uint32_t us)
{
    delay_us(us);
}

/**
 * SSTV micros callback - get microsecond counter
 */
static uint32_t sstv_micros_callback(void)
{
    return micros();
}

/**
 * Delay in microseconds using TIM3
 */
static void delay_us(uint32_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim3);
    while ((__HAL_TIM_GET_COUNTER(&htim3) - start) < us);
}

/**
 * Get microseconds counter
 */
static uint32_t micros(void)
{
    return __HAL_TIM_GET_COUNTER(&htim3);
}

/**
 * System Clock Configuration
 * Configure for 72 MHz using HSE (8 MHz) with PLL
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure HSE and PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  /* 8 MHz * 9 = 72 MHz */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* Configure system, AHB and APB clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     /* 72 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;      /* 36 MHz (max for APB1) */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;      /* 72 MHz */

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * GPIO Initialization
 */
static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure NSS pin (PA4) as output, high by default */
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = LORA_NSS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LORA_NSS_PORT, &GPIO_InitStruct);

    /* Configure RST pin (PB0) as output, high by default */
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = LORA_RST_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LORA_RST_PORT, &GPIO_InitStruct);

    /* Configure DIO0 pin (PA1) as input */
    GPIO_InitStruct.Pin = LORA_DIO0_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LORA_DIO0_PORT, &GPIO_InitStruct);

    /* Note: PB10 (tone output) is configured in TIM2_Init as alternate function */

    /* Configure LED pin (PC13) as output */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
}

/**
 * SPI1 Initialization
 */
static void SPI1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable SPI1 clock */
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* Configure SPI1 pins: SCK (PA5), MISO (PA6), MOSI (PA7) */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7;  /* SCK, MOSI */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;  /* MISO */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure SPI1 */
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;  /* 72MHz/8 = 9MHz */
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * TIM2 Initialization for tone generation (PWM on PB10 = TIM2_CH3)
 * This generates the audio tone signal connected to DIO2 of SX1276
 */
static void TIM2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* Enable TIM2 clock */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* Configure PB10 as TIM2_CH3 (alternate function)
     * Need to enable AFIO and remap TIM2 */
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* Partial remap of TIM2: CH3 on PB10 */
    __HAL_AFIO_REMAP_TIM2_PARTIAL_2();

    GPIO_InitStruct.Pin = TONE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TONE_PORT, &GPIO_InitStruct);

    /* Configure TIM2 for PWM
     * APB1 timer clock = 72 MHz (because APB1 prescaler is 2, timer gets x2)
     * Prescaler = 71 -> Timer clock = 1 MHz
     * For 1900 Hz tone: Period = 1000000 / 1900 - 1 = 525 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;              /* 72 MHz / 72 = 1 MHz */
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 525;                /* Default for ~1900 Hz */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    /* Configure PWM channel 3 */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 262;                  /* 50% duty cycle */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * TIM3 Initialization for microsecond timing
 * Timer runs at 1 MHz (1 us per tick)
 */
static void TIM3_Init(void)
{
    /* Enable TIM3 clock */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Configure TIM3 for 1 us resolution
     * APB1 timer clock = 72 MHz
     * Prescaler = 71 -> 1 MHz */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 71;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF;             /* 16-bit counter, wraps at 65535 us */
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * Error Handler
 */
static void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* Blink LED on PC13 */
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(500);
    }
}

/**
 * SysTick Handler (required by HAL)
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}
///////////////////////////////for SPI2/////////////////////////////////
static void MX_SPI2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable SPI1 clock */
    __HAL_RCC_SPI2_CLK_ENABLE();
  /* SPI2 parameter configuration*/
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_15;  /* SCK, MOSI */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;  /* MISO */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    ////////////////////////////////////

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}