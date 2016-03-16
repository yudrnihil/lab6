/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "lcd.h"
#include "i2c_ee.h"
#include "math.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup ADC_ADC1_DMA
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define HMC5883L_Addr	0x3C
#define L3G4200_Addr	0xD2
#define BMP085_Addr	0xEE
#define ADXL345_Addr	0xA6

#define PI 3.14159265358979323846

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
I2C_InitTypeDef I2C_InitStructure;
ErrorStatus HSEStartUpStatus;
uint16_t i;

int16_t x;
int16_t y;
u8 angleChar[3];

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void HMC_Init(void);
void LongDelay(u32 nCount);
void Delayms(u32 m);
uint16_t getHMCAngle();
/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  
  SystemInit();

  /* Enable FSMC, GPIOA, GPIOD, GPIOE, GPIOF, GPIOG and AFIO clocks */

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | 
                         RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO, ENABLE);
  
  STM3210E_LCD_Init(); 


  RCC_Configuration();    // RCC Init
  I2C_EE_Init();          // I2C Init
  HMC_Init();             // HMC Init
  
  while (1)
  {
    /* Please add code below to complete the LAB6 */
    /* You might want to create your own functions */
    uint16_t angle = getHMCAngle();
    angleChar[0] = HexValueOffset[angle/100];
    angleChar[1] = HexValueOffset[(angle%100)/10];
    angleChar[2] = HexValueOffset[angle%10];
    LCD_DrawString(0, 0, "Angle:", 6);
    LCD_DrawString(2, 0, angleChar, 3);
    
	


    /* main code ends here */    	
    Delayms(100); 
  }
}

void HMC_Init(void)
{
  I2C_ByteWrite(HMC5883L_Addr, 0x00, 0x70);
  I2C_ByteWrite(HMC5883L_Addr, 0x01, 0xA0);
  Delayms(10); 
}


void RCC_Configuration(void)
{
    /* RCC system reset(for debug purpose) */
//  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  { 
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* ADCCLK = PCLK2/4 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
  
#ifndef STM32F10X_CL  
    /* PLLCLK = 8MHz * 7 = 56 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_7);

#else
    /* Configure PLLs *********************************************************/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    RCC_PREDIV2Config(RCC_PREDIV2_Div5);
    RCC_PLL2Config(RCC_PLL2Mul_8);

    /* Enable PLL2 */
    RCC_PLL2Cmd(ENABLE);

    /* Wait till PLL2 is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
    {}

    /* PLL configuration: PLLCLK = (PLL2 / 5) * 7 = 56 MHz */ 
    RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_7);
#endif

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }

/* Enable peripheral clocks --------------------------------------------------*/
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
}
/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
}
#endif

void LongDelay(u32 nCount)
{
  for(; nCount != 0; nCount--);
}

void Delayms(u32 m)
{
  u32 i;
  
  for(; m != 0; m--)	
       for (i=0; i<50000; i++);
}

/**
 *get x, y value from HMC5883 and calculate the angle. The angle is in degrees 
 *0 ~ 359
 */
uint16_t getHMCAngle(){
  uint8_t hi, lo;
  double angle;
  I2C_ByteWrite(0x3C, 0x02, 0x01);
  Delayms(10);
  hi = I2C_ByteRead(0x3C, 0x03);
  lo = I2C_ByteRead(0x3C, 0x04);
  x = (hi << 8) & 0xff00 | lo;
  hi = I2C_ByteRead(0x3C, 0x07);
  lo = I2C_ByteRead(0x3C, 0x08);
  y = (hi << 8) & 0xff00 | lo;
  angle = atan((double)y/x)* 180 / PI;
  if (x < 0){
    return (uint16_t)(angle + 180);
  }
  if (x > 0 && y < 0){
    return (int)(angle + 360);
  }
  return (uint16_t)(angle);
}
  
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
