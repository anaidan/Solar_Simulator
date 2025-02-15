/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "constants.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef handle_GPDMA1_Channel7;

DAC_HandleTypeDef hdac1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void sendData(char* data);
void Read_ADC_Values(void);
void ADC_SelectChannel(uint32_t channel);
void clearDisplay();
void SPI_Transmit(uint8_t *txData, uint16_t len);
void drawLine(uint8_t lineNumber);
void toggle_VCOM();
void writeBuffer();
void init_buffer();
void clearDisplay();
void drawPixel(int16_t x, int16_t y, uint16_t color);
void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void drawFastRawVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void drawFastRawHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void fillScreen(uint16_t color);
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y);
void drawString(int16_t x, int16_t y, const char *str,  uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y);
void drawDouble(float val, int16_t x, int16_t y);
void drawInt(int val, int16_t x, int16_t y);
float map(float x, float in_min, float in_max, float out_min, float out_max);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
static inline int find_indices(const float array[], int n, float x);
static inline float linear_interpolate(float x0, float x1, float y1, float x, float slope);
static inline float limit(float in, float min, float max);
void curve_interp(uint16_t irradiance, float *V_sc_interp_inactive, float *Z_sc_interp_inactive);
void curve_interp_1d(float *x_orig, float *y_orig, uint8_t n_orig, float *x_new, float *y_new, uint8_t n_new);



volatile uint8_t slow_timer_flag = 0;
volatile uint16_t adc_buffer[5];

uint8_t Vcom;
uint8_t Line_cnt=0;
uint8_t sharpmem_buffer[400 * 240 / 8];
uint8_t rotation = 0;
volatile uint16_t DAC_test = 0;
volatile uint16_t cnt_pol = 0;

// Digital control loop test
uint32_t ADC_LOAD_VOL_CNTS = 0;
volatile float ADC_LOAD_VOL_V = 0;
volatile float ADC_VIN_V = 0;
volatile float ADC_LOAD_CUR_A = 0;
volatile float error = 0;
volatile float err_accum = 0;
volatile float REF_V = 0;
volatile float u = 0;
volatile uint16_t REF_CNTS = 0;

volatile uint16_t Z_target_ind = 0;
volatile float Z_V_slopes[45];
volatile float V_target = 0;
volatile float Z_out = 0;
volatile float Z_out_filt = 0;
volatile float Z_out_filt_prev = 0;

uint16_t ADC_POT_IRR_FILT = 0;
uint16_t ADC_POT_IRR_FILT_PREV = 0;

// 1<<n is a costly operation on AVR -- table usu. smaller & faster
static const uint8_t set[] = {1, 2, 4, 8, 16, 32, 64, 128};
static const uint8_t clr[] = {(uint8_t)~1,  (uint8_t)~2,  (uint8_t)~4,
                              (uint8_t)~8,  (uint8_t)~16, (uint8_t)~32,
                              (uint8_t)~64, (uint8_t)~128};


volatile uint16_t x1_mapped;
volatile uint16_t x2_mapped;
volatile uint16_t y1_mapped;
volatile uint16_t y2_mapped;
volatile uint16_t i = 0;

volatile float V_sc_interp_A[63];
volatile float Z_sc_interp_A[63];

volatile float V_sc_interp_B[63];
volatile float Z_sc_interp_B[63];

volatile float Z_V_slopes_A[63];
volatile float Z_V_slopes_B[63];

volatile uint8_t A_buffer_active = 1;
volatile uint8_t initial_run = 0;

uint32_t lastStableTime = 0; // Last time the value was stable
bool isStable = false;       // Flag to indicate stability
bool hasRun = false;         // Flag to ensure the code runs only once per stable value


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sendData(char* data)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
}

void ADC_SelectChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    // Configure the selected channel
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

uint32_t Read_ADC_Value(uint32_t channel)
{

	ADC_SelectChannel(channel);

    // Start the ADC conversion
    HAL_ADC_Start(&hadc1);

    // Wait for the conversion to complete (polling mode)
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)  // 10ms timeout
    {
        uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

    	return adcValue;
    }
    else
    {
        HAL_ADC_Stop(&hadc1);  // Make sure to stop the ADC conversion in case of error
    	return 0xFFFF;
    }
}

void SPI_Transmit(uint8_t *txData, uint16_t len)
{
    // Enable the chip select (CS) line if using software NSS
    //HAL_GPIO_WritePin(GPIOB, DISP_CS_Pin, GPIO_PIN_RESET);
    // Transmit data
    //uint16_t length = 1;//strlen((char *)txData);

    if (HAL_SPI_Transmit(&hspi2, txData, len, HAL_MAX_DELAY) != HAL_OK)
    {
        // Handle error
        Error_Handler();
    }

    // Disable the chip select (CS) line
    //HAL_GPIO_WritePin(GPIOB, DISP_CS_Pin, GPIO_PIN_SET);
}

void drawLine(uint8_t lineNumber) {

  HAL_GPIO_WritePin(GPIOB, DISP_CS_Pin, GPIO_PIN_SET);
  uint8_t line_cmd[] = {(0x01 | Vcom), lineNumber};
  //uint8_t temp = (0x01 | Vcom);
  SPI_Transmit(&line_cmd, 2); //write line command (bits reversed)
  //SPI_Transmit(&temp,1); //write line command (bits reversed)
  //SPI_Transmit(&lineNumber,1);
  uint8_t zerob = 0b00000000;
  for (int i = 0; i <50; i++) {
	  SPI_Transmit(&zerob, 1);
  }
  SPI_Transmit(&zerob, 1);
  SPI_Transmit(&zerob, 1);
  HAL_GPIO_WritePin(GPIOB, DISP_CS_Pin, GPIO_PIN_RESET);

}

void toggle_VCOM(){
  HAL_GPIO_WritePin(GPIOB, DISP_CS_Pin, GPIO_PIN_SET);
  Vcom = Vcom ? 0x00 : 0x02;
  uint8_t tog_cmd[] = {Vcom, 0x00};
  SPI_Transmit(&tog_cmd, 2); //write line command (bits reversed)
  HAL_GPIO_WritePin(GPIOB, DISP_CS_Pin, GPIO_PIN_RESET);
}

void writeBuffer(){
  uint16_t i, currentline;
  HAL_GPIO_WritePin(GPIOB, DISP_CS_Pin, GPIO_PIN_SET);

  uint8_t temp = (0x01 | Vcom);
  SPI_Transmit(&temp, 1);
  Vcom = Vcom ? 0x00 : 0x02;

  uint8_t bytes_per_line = 50;
  uint16_t totalbytes = 12000;

  for (i = 0; i < totalbytes; i += bytes_per_line) {
    uint8_t line[bytes_per_line + 2];

    // Send address byte
    currentline = ((i + 1) / (400 / 8)) + 1;
    line[0] = currentline;
    // copy over this line
    memcpy(line + 1, sharpmem_buffer + i, bytes_per_line);
    // Send end of line
    line[bytes_per_line + 1] = 0x00;
    // send it!
    //spidev->transfer(line, bytes_per_line + 2);
    SPI_Transmit(line, bytes_per_line + 2);
  }
  uint8_t zerob = 0b00000000;
  SPI_Transmit(&zerob, 1);

  HAL_GPIO_WritePin(GPIOB, DISP_CS_Pin, GPIO_PIN_RESET);

}

void init_buffer(){


	  // set all pixels in frame buffer to 1
	  for (i = 0; i < 12000; i++) {
		  sharpmem_buffer[i] = 255;
	  }

	 /* writeLine(30, 10, 30, 110, 0);
	  writeLine(30, 110, 260, 110, 0);

	  writeLine(30, 130, 30, 230, 0);
	  writeLine(30, 230, 260, 230, 0); */

	  writeLine(40, 50, 40, 200, 0);
	  writeLine(40, 200, 320, 200, 0);

	 /* for (i = 0; i < 44; i++) {
		  uint16_t x1_mapped =  map(V_sc[i], 0, 3, 30, 260);
		  uint16_t x2_mapped =  map(V_sc[i+1], 0, 3, 30, 260);
		  uint16_t y1_mapped =  map(V_sc[i]/Z_sc[i], 0, .075, 110, 10);
		  uint16_t y2_mapped =  map(V_sc[i+1]/Z_sc[i+1], 0, .075, 110, 10);

		  writeLine(x1_mapped, y1_mapped, x2_mapped, y2_mapped, 0);
	  }*/

	  for (i = 0; i < 60; i++) {


			  if(A_buffer_active == 1){
				  x1_mapped =  map(V_sc_interp_A[i], 0, 5, 40, 320);
				  x2_mapped =  map(V_sc_interp_A[i+1], 0, 5, 40, 320);
				  y1_mapped =  map(V_sc_interp_A[i]/Z_sc_interp_A[i], 0, .05, 200, 50);
				  y2_mapped =  map(V_sc_interp_A[i+1]/Z_sc_interp_A[i+1], 0, .05, 200, 50);
			  }
			  else{
				  x1_mapped =  map(V_sc_interp_B[i], 0, 5, 40, 320);
				  x2_mapped =  map(V_sc_interp_B[i+1], 0, 5, 40, 320);
				  y1_mapped =  map(V_sc_interp_B[i]/Z_sc_interp_B[i], 0, .05, 200, 50);
				  y2_mapped =  map(V_sc_interp_B[i+1]/Z_sc_interp_B[i+1], 0, .05, 200, 50);
			  }
		  writeLine(x1_mapped, y1_mapped, x2_mapped, y2_mapped, 0);
	  }

	  drawString(120,210,"Output Voltage [V]", 0, 1, 1, 1);
	  rotation = 3;
	  drawString(60,25,"Output Current [mA]", 0, 1, 1, 1);
	  rotation = 0;
	  //writeLine(19)
	  /*drawPixel(101,100,0);
	  writeLine(100,100,150,150,0);
	  drawCircle(250,100, 50, 0);
	  fillRect(100,50,50,50,0);
	  drawChar(25, 25, 'a', 0, 1, 2, 2);
	  drawString(25, 50, "Vout=2.436 V",  0, 1, 1, 1);

*/
}

void clearDisplay() {
  // MLCD write line command 0x80, reversed 0x01
  // MLCD clear memory command 0x20, reversed 0x04
  // MLCD static mode command 0x00
  // MLCD VCOM bit 0x40 , reversed 0x02
  HAL_GPIO_WritePin(GPIOB, DISP_CS_Pin, GPIO_PIN_SET);
  uint8_t clear_cmd[] = {(0x04 | Vcom), 0};
  SPI_Transmit(&clear_cmd, 2);
  Vcom = Vcom ? 0x00 : 0x02;
  //uint8_t temp = (0x04 | Vcom);
  //SPI_Transmit(&temp);
  //uint8_t zerob = 0b00000000;
  //SPI_Transmit(&zerob);
  HAL_GPIO_WritePin(GPIOB, DISP_CS_Pin, GPIO_PIN_RESET);
}

void drawPixel(int16_t x, int16_t y, uint16_t color){
	int16_t t;

    switch (rotation) {
    case 1:
      t = x;
      x = 400 - 1 - y;
      y = t;
      break;
    case 2:
      x = 400 - 1 - x;
      y = 240 - 1 - y;
      break;
    case 3:
      t = x;
      x = y;
      y = 240 - 1 - t;
      break;
    }

	if ((x < 0) || (x >= 400) || (y < 0) || (y >= 240))
	    return;

	  if (color) {
	    sharpmem_buffer[(y * 400 + x) / 8] |= set[x & 7];
	  } else {
	    sharpmem_buffer[(y * 400 + x) / 8] &= clr[x & 7];
	  }
}

void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {

  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1) {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0 <= x1; x0++) {
    if (steep) {
      drawPixel(y0, x0, color);
    } else {
      drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {

  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  drawPixel(x0, y0 + r, color);
  drawPixel(x0, y0 - r, color);
  drawPixel(x0 + r, y0, color);
  drawPixel(x0 - r, y0, color);

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 + x, y0 - y, color);
    drawPixel(x0 - x, y0 - y, color);
    drawPixel(x0 + y, y0 + x, color);
    drawPixel(x0 - y, y0 + x, color);
    drawPixel(x0 + y, y0 - x, color);
    drawPixel(x0 - y, y0 - x, color);
  }
}

void drawFastRawVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
	  // x & y already in raw (rotation 0) coordinates, no need to transform.
	  int16_t row_bytes = ((400 + 7) / 8);
	  uint8_t *ptr = &sharpmem_buffer[(x / 8) + y * row_bytes];

	  if (color > 0) {
	    uint8_t bit_mask = (0x80 >> (x & 7));
	    for (int16_t i = 0; i < h; i++) {
	      *ptr |= bit_mask;
	      ptr += row_bytes;
	    }
	  } else {
	    uint8_t bit_mask = ~(0x80 >> (x & 7));
	    for (int16_t i = 0; i < h; i++) {
	      *ptr &= bit_mask;
	      ptr += row_bytes;
	    }
	  }
}

void drawFastRawHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
  // x & y already in raw (rotation 0) coordinates, no need to transform.
  uint32_t buffer_index = y * 400 + x;
  for (uint32_t i = buffer_index; i < buffer_index + w; i++) {
	  sharpmem_buffer[i] = color;
  }
}

void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
  if (w < 0) { // Convert negative widths to positive equivalent
    w *= -1;
    x -= w - 1;
    if (x < 0) {
      w += x;
      x = 0;
    }
  }
  // Edge rejection (no-draw if totally off canvas)
  if ((y < 0) || (y >= height()) || (x >= width()) || ((x + w - 1) < 0)) {
    return;
  }

  if (x < 0) { // Clip left
    w += x;
    x = 0;
  }
  if (x + w >= width()) { // Clip right
    w = width() - x;
  }

  drawFastRawHLine(x, y, w, color);

}

void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
  /*if (h < 0) { // Convert negative heights to positive equivalent
    h *= -1;
    y -= h - 1;
    if (y < 0) {
      h += y;
      y = 0;
    }
  }

  // Edge rejection (no-draw if totally off canvas)
  if ((x < 0) || (x >= 400) || (y >= 240) || ((y + h - 1) < 0)) {
    return;
  }

  if (y < 0) { // Clip top
    h += y;
    y = 0;
  }
  if (y + h > 240) { // Clip bottom
    h = 240 - y;
  }

  drawFastRawVLine(x, y, h, color);*/
  writeLine(x, y, x, y + h - 1, color);
}

void fillScreen(uint16_t color) {
  if (sharpmem_buffer) {
    uint8_t hi = color >> 8, lo = color & 0xFF;
    if (hi == lo) {
      memset(sharpmem_buffer, lo, 400 * 240 * 2);
    } else {
      uint32_t i, pixels = 400 * 240;
      for (i = 0; i < pixels; i++)
    	 sharpmem_buffer[i] = color;
    }
  }
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  for (int16_t i = x; i < x + w; i++) {
    drawFastVLine(i, y, h, color);
  }
}

void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y) {


	if ((x >= 400) ||              // Clip right
		(y >= 240) ||             // Clip bottom
		((x + 6 * size_x - 1) < 0) || // Clip left
		((y + 8 * size_y - 1) < 0))   // Clip top
	  return;

	if ((c >= 176))
	  c++; // Handle 'classic' charset behavior

	for (int8_t i = 0; i < 5; i++) { // Char bitmap = 5 columns
	  uint8_t line = font[c * 5 + i];
	  for (int8_t j = 0; j < 8; j++, line >>= 1) {
		if (line & 1) {
		  if (size_x == 1 && size_y == 1)
			drawPixel(x + i, y + j, color);
		  else
			fillRect(x + i * size_x, y + j * size_y, size_x, size_y, color);
		} else if (bg != color) {
		  if (size_x == 1 && size_y == 1)
			  drawPixel(x + i, y + j, bg);
		  else
			fillRect(x + i * size_x, y + j * size_y, size_x, size_y, bg);
		}
	  }
	}
	if (bg != color) { // If opaque, draw vertical line for last column
	  if (size_x == 1 && size_y == 1)
		drawFastVLine(x + 5, y, 8, bg);
	  else
		fillRect(x + 5 * size_x, y, size_x, 8 * size_y, bg);
	}

}

void drawString(int16_t x, int16_t y, const char *str,  uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y){
	int16_t cursor_x = x;
	int16_t cursor_y = y;
	int16_t len = strlen(str);


	uint32_t i = 0;
	for(i; i<len; i++){
		drawChar(cursor_x, cursor_y, str[i], color, bg, size_x, size_y);
		cursor_x += 6;
	}

}

void drawDouble(float val, int16_t x, int16_t y){
	char count_str[50];
	sprintf(count_str, "%.3f", val);
	drawString(x, y, count_str,  0, 1, 1, 1);
}

void drawInt(int val, int16_t x, int16_t y){
	char count_str[50];
	sprintf(count_str, "%d", val);
	drawString(x, y, count_str,  0, 1, 1, 1);
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
    // Handle division by zero and avoid NaN results
    if (in_max - in_min == 0) {
        return 0; // Return a default value
    }
    // Map the input value to the new range
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Timer callback implementation
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)  // Check if interrupt is from TIM1
    {
        slow_timer_flag = 1;  // Set the flag
    }
    else if (htim->Instance == TIM3)
    {
    	HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_SET);
    	//HAL_GPIO_TogglePin(GPIOB, LED_RED_Pin);
    	//ADC_LOAD_VOL_CNTS = Read_ADC_Value(ADC_CHANNEL_1);
    	//ADC_LOAD_VOL_V    = (double)ADC_LOAD_VOL_CNTS/4095.0*2.5*6.0;
/*
    	error = 3 - ADC_LOAD_VOL_V;

        u =  .001*error + .01*err_accum;
        err_accum = err_accum + error * 1e-4;
        //Int_1 = limit(Int_1, -1, .0465);

    	REF_V = 2.5 - (u/2.5*4095.0)*2.5/4095.0;
    	REF_CNTS = (uint16_t)REF_V;
    	//VREF = 2.5 - int(VREF/2.5*4095)*2.5/4095;


    	if(DAC_test == 4000){
    		cnt_pol = -1;
    	}

    	else if(DAC_test == 2000 ){
    		cnt_pol = 1;
    	}

    	DAC_test+=cnt_pol;

		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_test);
    	HAL_GPIO_TogglePin(GPIOB, LED_RED_Pin);*/

    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
    	//HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_SET);
        ADC_LOAD_VOL_V  = (float)adc_buffer[0]*0.00366300366; ///4095.0*2.5*6.0
		ADC_LOAD_CUR_A  = ((float)adc_buffer[1]*0.00015262515 - 0.031095); ///4095.0*2.5 - 0.12438)/0.20/20.0

	    Z_out = ADC_LOAD_VOL_V / ADC_LOAD_CUR_A;
	    Z_out = limit(Z_out, .25, 22e3);

	    //alpha = calculate_alpha(1000, 50e3);
	    Z_out_filt = 0.38586954509 * Z_out + (0.6141304549)*Z_out_filt_prev;
	    Z_out_filt_prev = Z_out_filt;

        // A_buffer being used in high speed loop
    	if(A_buffer_active == 1 && initial_run==1){
    		Z_target_ind = find_indices(Z_sc_interp_A, 63, Z_out);
    		//Z_target_ind = 50;
    		V_target = linear_interpolate(Z_sc_interp_A[Z_target_ind], Z_sc_interp_A[Z_target_ind-1], V_sc_interp_A[Z_target_ind], Z_out, Z_V_slopes_A[Z_target_ind]);
    		//ADC_LOAD_VOL_V = V_target;

    	}

    	// B_buffer being used in high speed loop
    	else if(A_buffer_active == 0 && initial_run==1){
    		Z_target_ind = find_indices(Z_sc_interp_B, 63, Z_out);
    		//Z_target_ind = 50;
    		V_target = linear_interpolate(Z_sc_interp_B[Z_target_ind], Z_sc_interp_B[Z_target_ind-1], V_sc_interp_B[Z_target_ind], Z_out, Z_V_slopes_B[Z_target_ind]);
    	}

    	else{
    		V_target = 0.1;
    	}

    	//Z_target_ind = find_indices(Z_sc, 46, Z_out);
    	//V_target = linear_interpolate(Z_sc[Z_target_ind], Z_sc[Z_target_ind-1], V_sc[Z_target_ind], Z_out, Z_V_slopes[Z_target_ind]);

    	V_target = limit(V_target, 0.1, 7);

    	error = V_target - ADC_LOAD_VOL_V;
        if((error < 2e-3) & (error>0)){
        	error = 0;
        }
        else if((error > -2e-3) & (error<0)){
        	error = 0;
        }

        u =  .3*error + 20*err_accum;
        err_accum = err_accum + error * 1e-4;
        //Int_1 = limit(Int_1, -1, .0465);

        //ADC_VIN_V = V_target;

    	REF_V = 2.5 - u;
    	REF_CNTS = (uint16_t)(REF_V*1638); // /2.5*4095.0
    	if(REF_CNTS>4095)
    		REF_CNTS = 4095;
    	else if(REF_CNTS<0)
    		REF_CNTS = 0;

    	//ADC_VIN_V = REF_CNTS;

		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, REF_CNTS);
    	HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_RESET);

    	//HAL_GPIO_TogglePin(GPIOB, LED_RED_Pin);
    	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, 5);

    }
}

static inline int find_indices(const float array[], int n, float x) {
    uint8_t i = 1;
	for (i; i < n; i++) {
        if (array[i-1] >= x && x > array[i]) {
            return i;
        }
    }
    // If no valid indices are found
    return -1;
}

static inline float limit(float in, float min, float max){
   if(in>max){
      in=max;
   }
   else if(in<min){
      in = min;
   }
   return in;
}


static inline float linear_interpolate(float x0, float x1, float y0, float x, float slope){

    return y0 + slope * (x - x0);
}

void precompute_slopes(const float *x_values, const float *y_values, float *slopes, int size) {

	for (uint8_t i = 1; i < size; i++) {
        float dx = x_values[i - 1] - x_values[i];
        float dy = y_values[i - 1] - y_values[i];

        // Avoid division by zero
        if (dx != 0.0f) {
            slopes[i] = dy / dx;
        } else {
            slopes[i] = 0.0f;  // Handle vertical segments
        }
    }
}


void curve_interp_1d(float *x_orig, float *y_orig, uint8_t n_orig, float *x_new, float *y_new, uint8_t n_new)
{
    for (uint8_t i = 0; i < n_new; i++) {
    	float x = x_new[i];

        // Find the interval [x_orig[j], x_orig[j+1]] where x falls
    	uint8_t j;
        for (j = 0; j < n_orig - 1; j++) {
            if (x_orig[j] <= x && x <= x_orig[j + 1]) {
                break;
            }
        }

        // Handle edge cases where x is outside the range
        if (j == n_orig - 1 || x < x_orig[0]) {
            y_new[i] = (x < x_orig[0]) ? y_orig[0] : y_orig[n_orig - 1];
            continue;
        }

        // Linear interpolation formula
        float x1 = x_orig[j], y1 = y_orig[j];
        float x2 = x_orig[j + 1], y2 = y_orig[j + 1];
        y_new[i] = y1 + (y2 - y1) / (x2 - x1) * (x - x1);
    }
}


void curve_interp(uint16_t irradiance, float *V_sc_interp_inactive, float *Z_sc_interp_inactive){

	char buffert[50];

	uint16_t curve_num_low  = (uint16_t)floor((float)irradiance / 100);
	uint16_t curve_num_high = (uint16_t)ceil((float)irradiance / 100);
	float alpha = ((float)(irradiance - curve_num_low*100))/100;


	float x_max_1 = V_max[curve_num_low-1];
	float x_max_2 = V_max[curve_num_high-1];
	float x_max_new = x_max_1 + (x_max_2 - x_max_1) * alpha;

	//sprintf(buffert, "%.3f\r\n", alpha);
	//sendData(buffert);

	float x1_norm[62];
	float x2_norm[62];
	float y2_interp_norm[62];
	float y1;
	float y2[62];

	for(int i=0; i<62; i++){
		x1_norm[i] = V_sc_irr[i] / x_max_1;
		x2_norm[i] = V_sc_irr[i] / x_max_2;
		y2[i] = Z_sc_irr[(62 * (curve_num_high-1) + i)];
		//sprintf(buffert, "%.3f\r\n", y2[i]);
		//sendData(buffert);
	}

	curve_interp_1d(x2_norm, y2, 62, x1_norm, y2_interp_norm, 62);


	for(int i=0; i<62; i++){
		y1 = Z_sc_irr[(62 * (curve_num_low-1) + i)];
		Z_sc_interp_inactive[61-i] =  (1 - alpha) * y1 + alpha * y2_interp_norm[i];
		V_sc_interp_inactive[61-i] = x1_norm[i] * x_max_new;
		/*sprintf(buffert, "%.3f\r\n", Z_sc_interp_inactive[i]);
		//sprintf(buffert, "%.3f\r\n", y2_interp_norm[i]);
		sendData(buffert);*/
	}
	V_sc_interp_inactive[62] = 0;
	Z_sc_interp_inactive[62] = 0;
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
  MX_GPDMA1_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_RESET);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_GPIO_WritePin(GPIOB, DISP_CS_Pin, GPIO_PIN_RESET);
  clearDisplay();
  init_buffer();
  writeBuffer();

  precompute_slopes(Z_sc, V_sc, Z_V_slopes, 46);


  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1); // Start DAC

  HAL_TIM_Base_Start_IT(&htim1);  // Slow LCD interrupt (T = 300ms)
  HAL_TIM_Base_Start_IT(&htim3);  // Fast control loop interrupt (f = 10kHz)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, 5);


  //init_buffer();
  //toggle_VCOM();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(slow_timer_flag){
		  //HAL_DAC_Start(&hdac1, DAC_CHANNEL_1); // Start DAC
		  //HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2300);
		  //sendData("Hello, ST-Link VCP!\n");
		  //uint32_t ADC_VIN_CNTS       = Read_ADC_Value(ADC_CHANNEL_19);
		  //ADC_LOAD_VOL_CNTS           = Read_ADC_Value(ADC_CHANNEL_1);
		  //uint32_t ADC_LOAD_CURR_CNTS = Read_ADC_Value(ADC_CHANNEL_0);
		  //uint32_t ADC_POT_CNTS       = Read_ADC_Value(ADC_CHANNEL_14);
		  //uint32_t ADC_TEMP_CNTS      = Read_ADC_Value(ADC_CHANNEL_9);

		  ADC_VIN_V       = (float)adc_buffer[2]/4095.0*2.5*6.1;
		  //ADC_LOAD_VOL_V         = (double)ADC_LOAD_VOL_CNTS/4095.0*2.5*6.0;
		  //double ADC_LOAD_CUR_A  = ((double)adc_buffer[1]/4095.0*2.5 - 0.12438)/0.20/20.0;
		  float ADC_TEMP_DC     = ((float)adc_buffer[3]/4095.0*2.5 - 0.5)/0.010;
		  uint16_t ADC_POT_IRR  = (uint16_t)((float)adc_buffer[4]*0.2442002442);

		  ADC_POT_IRR_FILT = (uint16_t)((.75 * (float)ADC_POT_IRR) + (0.25 * (float)ADC_POT_IRR_FILT_PREV));

		  	 // *******************
		  uint32_t currentTime = HAL_GetTick(); // Current system time in ms

		  //if(abs(ADC_POT_IRR_FILT_PREV - ADC_POT_IRR_FILT)  > 3){
		  if(ADC_POT_IRR_FILT != ADC_POT_IRR_FILT_PREV){
			  lastStableTime = currentTime;
			  isStable = false;
			  hasRun = false;
		  } else if (!isStable && (currentTime - lastStableTime >= 300)) {
		      // Value remained unchanged for the required duration
			  isStable = true;
		    }
		    if (isStable && !hasRun) {
		        hasRun = true; // Ensure this block only runs once
		        HAL_GPIO_TogglePin(GPIOB, LED_YELLOW_Pin);


		            // A_buffer being used in high speed loop
		        	if(A_buffer_active == 1){
		        		curve_interp(ADC_POT_IRR_FILT, V_sc_interp_B, Z_sc_interp_B);
		        		precompute_slopes(Z_sc_interp_B, V_sc_interp_B, Z_V_slopes_B, 63);
		        		A_buffer_active = 0;

		        	}

		        	// B_buffer being used in high speed loop
		        	else{
		        		curve_interp(ADC_POT_IRR_FILT, V_sc_interp_A, Z_sc_interp_A);
		        		precompute_slopes(Z_sc_interp_A, V_sc_interp_A, Z_V_slopes_A, 63);
		        		A_buffer_active = 1;
		        	}

		        //

				initial_run = 1;

		    } //**************************
		    ADC_POT_IRR_FILT_PREV = ADC_POT_IRR_FILT;



		  /*char adcBuffer1[12];
		  sprintf(adcBuffer1, "%lu\n", ADC_VIN_CNTS);  // Convert to string (optional)

		  sendData(adcBuffer1);
		  sprintf(adcBuffer1, "%lu\n\n", ADC_LOAD_CURR_CNTS);  // Convert to string (optional)
		  sendData(adcBuffer1);*/

		  uint8_t OUTPUT_EN_SW_STATE =  HAL_GPIO_ReadPin(GPIOA, OUTPUT_EN_SW_Pin);
		  HAL_GPIO_WritePin(GPIOA, EN_Pin, OUTPUT_EN_SW_STATE);

		  HAL_GPIO_TogglePin(GPIOB, LED_BLUE_Pin);

		  init_buffer();

		  drawString(290, 20, "Vout = ",  0, 1, 1, 1);
		  drawDouble(ADC_LOAD_VOL_V, 330, 20);
		  drawString(365, 20, "V",  0, 1, 1, 1);

		  drawString(290, 35, "Iout = ",  0, 1, 1, 1);
		  drawDouble(ADC_LOAD_CUR_A*1000, 330, 35);
		  drawString(370, 35, "mA",  0, 1, 1, 1);

		  drawString(290, 50, "Temp = ",  0, 1, 1, 1);
		  drawDouble(ADC_TEMP_DC, 330, 50);
		  drawString(370, 50, "C",  0, 1, 1, 1);

		  drawString(290, 65, "Vin = ",  0, 1, 1, 1);
		  drawDouble(ADC_VIN_V, 330, 65);
		  drawString(365, 65, "V",  0, 1, 1, 1);

		  drawString(290, 80, "Irr = ",  0, 1, 1, 1);
		  drawInt(ADC_POT_IRR_FILT, 330, 80);
		  drawString(350, 80, "W/m^2",  0, 1, 1, 1);

		  //
		  //uint16_t ADC_LOAD_VOL_MAPPED   = (uint16_t)roundf(map(ADC_LOAD_VOL_V, 0, 3, 30, 260));
		  uint16_t ADC_LOAD_VOL_MAPPED   = (uint16_t)roundf(map(ADC_LOAD_VOL_V, 0, 5, 40, 320));
		  uint16_t ADC_LOAD_CUR_A_MAPPED = (uint16_t)roundf(map(ADC_LOAD_CUR_A, 0, .05, 200, 50));
		  drawCircle(ADC_LOAD_VOL_MAPPED, ADC_LOAD_CUR_A_MAPPED, 2, 0);

		  writeBuffer();

		  slow_timer_flag = 0;
		  //HAL_Delay(300);
	  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 26;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 16;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_19;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel7_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 519;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 59999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
  hpcd_USB_DRD_FS.Init.speed = USBD_FS_SPEED;
  hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DISP_CS_Pin|LED_RED_Pin|LED_BLUE_Pin|LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : EN_Pin */
  GPIO_InitStruct.Pin = EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DISP_CS_Pin LED_RED_Pin LED_BLUE_Pin LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = DISP_CS_Pin|LED_RED_Pin|LED_BLUE_Pin|LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OUTPUT_EN_SW_Pin */
  GPIO_InitStruct.Pin = OUTPUT_EN_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OUTPUT_EN_SW_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
