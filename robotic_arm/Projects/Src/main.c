/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 09/10/2014 11:13:03
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "xnucleoihm02a1.h"
#include "example.h"
#include "example_usart.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_tim.h"
#include "xnucleoihm02a1_interface.h"
#include "stdlib.h"
#include "string.h"
//#include "main.h"
//#include "i2c-communication.h"

#define HOME_SPEED 30000 // conveyor speed
#define M_PI 3.14159265358979323846

#define STEPS_PER_RAD_BASE 38003.0 	// 5 2/11 gearbox from 1/128th microstepping motor, 10/18 belt reduction
#define STEPS_PER_RAD_HUM  111911.0   // 13 212/289 gearbox from 1/128th microstepping motor, 10/20 belt reduction
#define STEPS_PER_RAD_ULN 115000.0  // same, for now
#define STEPS_PER_RAD_WRIST 111911.0  // same, for now

//---------------------uart------------------------
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
UART_HandleTypeDef huart2;
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
//--------------------------------------------------

//--------------motor encoder init------------------
TIM_Encoder_InitTypeDef encoder_base;
TIM_Encoder_InitTypeDef encoder_shoulder;
TIM_Encoder_InitTypeDef encoder_elbow;
TIM_Encoder_InitTypeDef encoder_wrist;
//--------------------------------------------------
//GPIO_InitTypeDef GPIO_InitStructure;


//--------------------------------------------------

void SystemClock_Config(void);

#define MICROSTEPPING_MOTOR_EXAMPLE        //!< Uncomment to performe the standalone example
//#define MICROSTEPPING_MOTOR_USART_EXAMPLE  //!< Uncomment to performe the USART example
#if ((defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option only!"
#elif ((!defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (!defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option!"
#endif
#if (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE) && (!defined (NUCLEO_USE_USART)))
  #error "Please define "NUCLEO_USE_USART" in "stm32fxxx_x-nucleo-ihm02a1.h"!"
#endif
	
	/* Private Variables ----------------------*/

/* Variable used to get converted value */
__IO uint16_t uhADCxConvertedValue = 0;

uint16_t CurrBoard = 0;

/* Private function prototypes -----------------------------------------------*/
//static void SystemClock_Config(void);
static void Error_Handler(void);
uint16_t Read_ADC(void);



void SerialPutc2(char c){
	while((huart2.Instance->SR & USART_SR_TXE) == 0)
		;
	huart2.Instance->DR = c;
}

void SerialPuts2(char *ptr){
	while(*ptr)
			SerialPutc2(*ptr++);
}

void SerialPutc6(char c){
	while((huart6.Instance->SR & USART_SR_TXE) == 0)
		;
	huart6.Instance->DR = c;
}

void SerialPuts6(char *ptr){
	while(*ptr)
			SerialPutc6(*ptr++);
}

char SerialGetc6(){
	while((huart6.Instance->SR & USART_SR_RXNE) == 0)
		;
	return huart6.Instance->DR;
}

//equations from matrices
void MoveNextPoint(float* currentAngles, float* nextPoint){

		HAL_Init();																									
	
		HAL_Delay(1000);

		__HAL_RCC_GPIOA_CLK_ENABLE();
  
		StepperMotorBoardHandle_t *StepperMotorBoardHandle;
		MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;
	
		uint8_t board1 = EXPBRD_ID(0);
		uint8_t board2 = EXPBRD_ID(1);
		uint8_t board3 = EXPBRD_ID(2);
	
		MotorParameterDataGlobal = GetMotorParameterInitData();
  
		for (uint8_t id = 0; id < EXPBRD_MOUNTED_NR; id++)
		{
			StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(id));
			MotorParameterDataSingle = MotorParameterDataGlobal+(id*L6470DAISYCHAINSIZE);
			StepperMotorBoardHandle->Config(MotorParameterDataSingle);
		}

	
		float angle01 = 0, angle12 = 0, angle23 = 0, angle34 = 0, angle45 = 0;

		float d_angle01 = 0, d_angle12 = 0, d_angle23 = 0, d_angle34 = 0, d_angle45 = 0;
	
		float c_angle01 = currentAngles[0], c_angle12 = currentAngles[1], c_angle23 = currentAngles[2], c_angle34 = currentAngles[3], c_angle45 = currentAngles[4];
	
    float px = nextPoint[0], py = nextPoint[1], pz = nextPoint[2] + 152.4 - 45;

    float h_12 = 302, h_23 = 199;

    float ux = 0, uy = 0, uz = -1;

    float vx = 0, vy = 1, vz = 0;

    float wx = 1, wy = 0, wz = 0;

    float asin_num_frac1 = 0, asin_num_frac2 = 0, asin_num_frac3 = 0, asin_denom_frac1 = 0, asin_denom_frac2 = 0;
	
		//-------------------------------------------------- ANGLE CALCULATIONS -------------------------------------------------
		//base
		angle01 = atan2(py,px);

		//wrist rotation
    angle45 = -atan2(vz,uz);

		//humerus
    asin_num_frac1 = pow(sin(angle01)*py*cos(angle01) - pow(sin(angle01),2)*px + px,2)/pow(cos(angle01)*h_23,2);

    asin_num_frac2 = pow(pz,2)/pow(h_23,2);

    asin_num_frac3 = pow(h_12,2)/pow(h_23,2);

    asin_denom_frac1 = 4*pow(sin(angle01)*py*cos(angle01) - pow(sin(angle01),2)*px + px, 2)*pow(h_12,2)/(pow(cos(angle01),2)*pow(h_23,4));

    asin_denom_frac2 = 4*pow(pz,2)*pow(h_12,2)/pow(h_23,4);

    
    angle12 = -atan2(sin(angle01)*py*cos(angle01) - pow(sin(angle01),2)*px + px, cos(angle01)*pz) + asin((asin_num_frac1 + asin_num_frac2 + asin_num_frac3 - 1)/pow(asin_denom_frac1 + asin_denom_frac2, 0.5));

    if(angle12 < 0.16) // the height of the limit switch
			angle12 = -atan2(sin(angle01)*py*cos(angle01) - pow(sin(angle01),2)*px + px, cos(angle01)*pz) + (M_PI - asin((asin_num_frac1 + asin_num_frac2 + asin_num_frac3 - 1)/pow(asin_denom_frac1 + asin_denom_frac2, 0.5)));


    //elbow
    angle23 = -atan2((sin(angle12)*h_12 - pz)*cos(angle01), sin(angle01)*py*cos(angle01) - cos(angle01)*cos(angle12)*h_12 - pow(sin(angle01),2)*px + px) - angle12;

		//wrist
    angle34 = -atan2(sin(angle01)*wy*cos(angle01) - pow(sin(angle01),2)*wx + wx, cos(angle01)*wz) - angle23 - angle12;
	
		//--------------------------------------------------------D_ANGLE---------------------------------------------------------
		
		d_angle01 = angle01 - c_angle01;
		d_angle12 = angle12 - c_angle12;
		d_angle23 = angle23 + d_angle12 - c_angle23;
		d_angle34 = 0; //angle34 - c_angle34;
		d_angle45 = angle45 - c_angle45;
		
		//---------------------------------------------------- ANGLE MOVEMENTS ---------------------------------------------------
		
		//base
		StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(0)); 
		StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(0));
	
		if(d_angle01 > 0)
			StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_BASE*d_angle01);
	
		else
			StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_BASE*d_angle01);
		
		c_angle01 = angle01;
		
		//shoulder
		StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(1));																								
		StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(1));
	
		if(d_angle12 > 0)																						
			StepperMotorBoardHandle->Command->Move(board1, L6470_ID(1), L6470_DIR_FWD_ID, STEPS_PER_RAD_HUM*d_angle12);
	
		else																							
			StepperMotorBoardHandle->Command->Move(board1, L6470_ID(1), L6470_DIR_REV_ID, STEPS_PER_RAD_HUM*d_angle12);
	
		c_angle12 = angle12;
		
	
		//elbow
		StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(0));
		StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(0));
	
		if(d_angle23 > 0)
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_ULN*d_angle23); 
	
		else
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_ULN*d_angle23); 
	
		c_angle23 = angle23;
		
		
		//wrist
		StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(1));
		StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(1));
	
		if(d_angle34 > 0)
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(1), L6470_DIR_REV_ID, STEPS_PER_RAD_WRIST*d_angle34); 
	
		else	
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(1), L6470_DIR_FWD_ID, STEPS_PER_RAD_WRIST*d_angle34); 
	
	
		c_angle34 = angle34;

		/*
		//wrist rotation
		StepperMotorBoardHandle->Command->SoftStop(board3, L6470_ID(0));
		StepperMotorBoardHandle->Command->ResetPos(board3, L6470_ID(0));
	
		if(d_angle34 > 0)
			StepperMotorBoardHandle->Command->Move(board3, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_WRIST*d_angle45); 
	
		else	
			StepperMotorBoardHandle->Command->Move(board3, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_WRIST*d_angle45); 
	
		//while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(1), BUSY_ID)==0);
	
		c_angle45 = angle45;
		*/
	
		currentAngles[0] = angle01;
		currentAngles[1] = angle12;
		currentAngles[2] = angle23;
		currentAngles[3] = angle34;
		currentAngles[4] = angle45;

}

int MoveNextPoint3(float* currentAngles, float* prevPoint, float* nextPoint, float orientation[3][3]){
		
		HAL_Init();																									

		__HAL_RCC_GPIOA_CLK_ENABLE();
  
		StepperMotorBoardHandle_t *StepperMotorBoardHandle;
		MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;
	
		uint8_t board1 = EXPBRD_ID(0);
		uint8_t board2 = EXPBRD_ID(1);
		uint8_t board3 = EXPBRD_ID(2);
	
		MotorParameterDataGlobal = GetMotorParameterInitData();
  
		for (uint8_t id = 0; id < EXPBRD_MOUNTED_NR; id++)
		{
			StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(id));
			MotorParameterDataSingle = MotorParameterDataGlobal+(id*L6470DAISYCHAINSIZE);
			StepperMotorBoardHandle->Config(MotorParameterDataSingle);
		}

		float angle01 = 0, angle12 = 0, angle23 = 0, angle34 = 0, angle45 = 0;

		float d_angle01 = 0, d_angle12 = 0, d_angle23 = 0, d_angle34 = 0, d_angle45 = 0;
	
		float c_angle01 = currentAngles[0], c_angle12 = currentAngles[1], c_angle23 = currentAngles[2], c_angle34 = currentAngles[3], c_angle45 = currentAngles[4];
	
    float px = nextPoint[0], py = nextPoint[1], pz = nextPoint[2]-45;

    float h_12 = 305, h_23 = 200;

    float ux = orientation[0][0], uy = orientation[0][1], uz = orientation[0][2];

    float vx = orientation[1][0], vy = orientation[1][1], vz = orientation[1][2];

    float wx = orientation[2][0], wy = orientation[2][1], wz = orientation[2][2];

    float asin_num_frac1 = 0, asin_num_frac2 = 0, asin_num_frac3 = 0, asin_denom_frac1 = 0, asin_denom_frac2 = 0;
	
		//-------------------------------------------------- ANGLE CALCULATIONS -------------------------------------------------
		
		//base
		angle01 = atan2(py,px);

		//wrist rotation
    angle45 = -atan2(vz,uz);

		//humerus
    
    angle12 =  atan2(pz,pow(pow(px,2)+pow(py,2),0.5)) + acos((pow(h_23,2) - pow(h_12,2) - pow(px,2) - pow(py,2) - pow(pz,2))/(-2*h_12*(pow(pow(px,2) + pow(py,2) + pow(pz,2),0.5))));

		float initial_mag = pow(pow(prevPoint[0],2) + pow(prevPoint[1],2) + pow(prevPoint[2],2),0.5), final_mag = pow(pow(nextPoint[0],2) + pow(nextPoint[1],2) + pow(nextPoint[2],2),0.5);
		
		angle23 = acos((pow(px,2)+pow(py,2)+pow(pz,2)-pow(h_12,2)-pow(h_23,2))/(-2*h_12*h_23));
		
    //elbow
		if(final_mag > initial_mag){ //if the final vector from origin to point is greater 
			if(-1*(M_PI-angle23) < currentAngles[2]) //and the final angle is smaller than the current angle
				angle23 = M_PI - acos((pow(px,2)+pow(py,2)+pow(pz,2)-pow(h_12,2)-pow(h_23,2))/(-2*h_12*h_23));	//take the supplementary angle
		}
		
		//wrist
    angle34 = -atan2(sin(angle01)*wy*cos(angle01) - pow(sin(angle01),2)*wx + wx, cos(angle01)*wz) - angle23 - angle12;
	
		//--------------------------------------------------------D_ANGLE---------------------------------------------------------
		
		d_angle01 = angle01 - c_angle01;
		
		/*
		//shortest path for base -- implement once wires are sorted
		if(d_angle01 > M_PI){
				d_angle01 = d_angle01 - 2*M_PI;
		}
		else if(d_angle01 < -M_PI){
				d_angle01 = 2*M_PI - d_angle01;
		}
		*/
		
		d_angle12 = angle12 - c_angle12;
		d_angle23 = -1*(M_PI - angle23) + d_angle12 - c_angle23;
		d_angle34 = 0; //angle34 - c_angle34;
		d_angle45 = angle45 - c_angle45;
		
		//---------------------------------------------------- ANGLE MOVEMENTS ---------------------------------------------------
		
		
		//base
		StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(0)); 
		StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(0));
	
		if(d_angle01 > 0)
			StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_BASE*d_angle01);
	
		else
			StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_BASE*d_angle01);
		
		c_angle01 = angle01;
		
	
		//shoulder
		StepperMotorBoardHandle->Command->HardStop(EXPBRD_ID(0), L6470_ID(1));																								
		//StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(1));
	
		if(d_angle12 > 0){																						
			StepperMotorBoardHandle->Command->Move(EXPBRD_ID(0), L6470_ID(1), L6470_DIR_FWD_ID, STEPS_PER_RAD_HUM*d_angle12);
		}
		
		else{																							
			StepperMotorBoardHandle->Command->Move(EXPBRD_ID(0), L6470_ID(1), L6470_DIR_REV_ID, STEPS_PER_RAD_HUM*d_angle12);
		}
		
		c_angle12 = angle12;
		
	
		//elbow
		StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(0));
		//StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(0));
	
		if(d_angle23 > 0){
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_ULN*d_angle23); 
		}
		
		else{
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_ULN*d_angle23); 
		}
		
		while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0));
		
		c_angle23 = angle23;
		
		/*
		//wrist
		StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(1));
		StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(1));
	
		if(d_angle34 > 0)
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(1), L6470_DIR_REV_ID, STEPS_PER_RAD_WRIST*d_angle34); 
	
		else	
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(1), L6470_DIR_FWD_ID, STEPS_PER_RAD_WRIST*d_angle34); 
	
		c_angle34 = angle34;
		*/
		/*
		//wrist rotation
		StepperMotorBoardHandle->Command->SoftStop(board3, L6470_ID(0));
		StepperMotorBoardHandle->Command->ResetPos(board3, L6470_ID(0));
	
		if(d_angle34 > 0)
			StepperMotorBoardHandle->Command->Move(board3, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_WRIST*d_angle45); 
	
		else	
			StepperMotorBoardHandle->Command->Move(board3, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_WRIST*d_angle45); 
	
		//while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(1), BUSY_ID)==0);
	
		c_angle45 = angle45;
		*/
	
		currentAngles[0] = angle01;
		currentAngles[1] = angle12;
		currentAngles[2] = -1*(M_PI - angle23);
		currentAngles[3] = angle34;
		currentAngles[4] = angle45;
		
		prevPoint[0] = nextPoint[0];
		prevPoint[1] = nextPoint[1];
		prevPoint[2] = nextPoint[2];
}

//equations from trig
int MoveNextPoint2(float* currentAngles, float* prevPoint, float* nextPoint){
		
		HAL_Init();																									
	
		//HAL_Delay(1000);

		__HAL_RCC_GPIOA_CLK_ENABLE();
  
		StepperMotorBoardHandle_t *StepperMotorBoardHandle;
		MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;
	
		uint8_t board1 = EXPBRD_ID(0);
		uint8_t board2 = EXPBRD_ID(1);
		uint8_t board3 = EXPBRD_ID(2);
	
		MotorParameterDataGlobal = GetMotorParameterInitData();
  
		for (uint8_t id = 0; id < EXPBRD_MOUNTED_NR; id++)
		{
			StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(id));
			MotorParameterDataSingle = MotorParameterDataGlobal+(id*L6470DAISYCHAINSIZE);
			StepperMotorBoardHandle->Config(MotorParameterDataSingle);
		}

		float angle01 = 0, angle12 = 0, angle23 = 0, angle34 = 0, angle45 = 0;

		float d_angle01 = 0, d_angle12 = 0, d_angle23 = 0, d_angle34 = 0, d_angle45 = 0;
	
		float c_angle01 = currentAngles[0], c_angle12 = currentAngles[1], c_angle23 = currentAngles[2], c_angle34 = currentAngles[3], c_angle45 = currentAngles[4];
	
    float px = nextPoint[0], py = nextPoint[1], pz = nextPoint[2]-45;

    float h_12 = 305, h_23 = 200;

    float ux = 0, uy = 0, uz = -1;

    float vx = 0, vy = 1, vz = 0;

    float wx = 1, wy = 0, wz = 0;

    float asin_num_frac1 = 0, asin_num_frac2 = 0, asin_num_frac3 = 0, asin_denom_frac1 = 0, asin_denom_frac2 = 0;
	
		//-------------------------------------------------- ANGLE CALCULATIONS -------------------------------------------------
		
		//base
		angle01 = atan2(py,px);

		//wrist rotation
    angle45 = -atan2(vz,uz);

		//humerus
    
    angle12 =  atan2(pz,pow(pow(px,2)+pow(py,2),0.5)) + acos((pow(h_23,2) - pow(h_12,2) - pow(px,2) - pow(py,2) - pow(pz,2))/(-2*h_12*(pow(pow(px,2) + pow(py,2) + pow(pz,2),0.5))));

		float initial_mag = pow(pow(prevPoint[0],2) + pow(prevPoint[1],2) + pow(prevPoint[2],2),0.5), final_mag = pow(pow(nextPoint[0],2) + pow(nextPoint[1],2) + pow(nextPoint[2],2),0.5);
		
		angle23 = acos((pow(px,2)+pow(py,2)+pow(pz,2)-pow(h_12,2)-pow(h_23,2))/(-2*h_12*h_23));
		
    //elbow
		if(final_mag > initial_mag){ //if the final vector from origin to point is greater 
			if(-1*(M_PI-angle23) < currentAngles[2]) //and the final angle is smaller than the current angle
				angle23 = M_PI - acos((pow(px,2)+pow(py,2)+pow(pz,2)-pow(h_12,2)-pow(h_23,2))/(-2*h_12*h_23));	//take the supplementary angle
		}
		
		//wrist
    angle34 = -atan2(sin(angle01)*wy*cos(angle01) - pow(sin(angle01),2)*wx + wx, cos(angle01)*wz) - angle23 - angle12;
	
		//--------------------------------------------------------D_ANGLE---------------------------------------------------------
		
		d_angle01 = angle01 - c_angle01;
		
		/*
		//shortest path for base -- implement once wires are sorted
		if(d_angle01 > M_PI){
				d_angle01 = d_angle01 - 2*M_PI;
		}
		else if(d_angle01 < -M_PI){
				d_angle01 = 2*M_PI - d_angle01;
		}
		*/
		
		d_angle12 = angle12 - c_angle12;
		d_angle23 = -1*(M_PI - angle23) + d_angle12 - c_angle23;
		d_angle34 = 0; //angle34 - c_angle34;
		d_angle45 = angle45 - c_angle45;
		
		//---------------------------------------------------- ANGLE MOVEMENTS ---------------------------------------------------
		
		//base
		StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(0)); 
		StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(0));
	
		if(d_angle01 > 0)
			StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_BASE*d_angle01);
	
		else
			StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_BASE*d_angle01);
		
		c_angle01 = angle01;
		while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0));
	
		//shoulder
		StepperMotorBoardHandle->Command->HardStop(EXPBRD_ID(0), L6470_ID(1));																								
		//StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(1));
	
		if(d_angle12 > 0){																						
			StepperMotorBoardHandle->Command->Move(EXPBRD_ID(0), L6470_ID(1), L6470_DIR_FWD_ID, STEPS_PER_RAD_HUM*d_angle12);
		}
		
		else{																							
			StepperMotorBoardHandle->Command->Move(EXPBRD_ID(0), L6470_ID(1), L6470_DIR_REV_ID, STEPS_PER_RAD_HUM*d_angle12);
		}
		
		c_angle12 = angle12;
		
	
		//elbow
		StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(0));
		//StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(0));
	
		if(d_angle23 > 0){
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_ULN*d_angle23); 
		}
		
		else{
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_ULN*d_angle23); 
		}
		
		while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0));
		
		c_angle23 = angle23;
		
		/*
		//wrist
		StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(1));
		StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(1));
	
		if(d_angle34 > 0)
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(1), L6470_DIR_REV_ID, STEPS_PER_RAD_WRIST*d_angle34); 
	
		else	
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(1), L6470_DIR_FWD_ID, STEPS_PER_RAD_WRIST*d_angle34); 
	
		c_angle34 = angle34;
		*/
		/*
		//wrist rotation
		StepperMotorBoardHandle->Command->SoftStop(board3, L6470_ID(0));
		StepperMotorBoardHandle->Command->ResetPos(board3, L6470_ID(0));
	
		if(d_angle34 > 0)
			StepperMotorBoardHandle->Command->Move(board3, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_WRIST*d_angle45); 
	
		else	
			StepperMotorBoardHandle->Command->Move(board3, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_WRIST*d_angle45); 
	
		//while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(1), BUSY_ID)==0);
	
		c_angle45 = angle45;
		*/
	
		currentAngles[0] = angle01;
		currentAngles[1] = angle12;
		currentAngles[2] = -1*(M_PI - angle23);
		currentAngles[3] = angle34;
		currentAngles[4] = angle45;
		
		prevPoint[0] = nextPoint[0];
		prevPoint[1] = nextPoint[1];
		prevPoint[2] = nextPoint[2];
}

//creates linear point path and runs movenextpoint2 along them
void LinearMove(float* currentPoint, float* finalPoint, float* currentAngles){
		
		HAL_Init();
	
		__HAL_RCC_GPIOA_CLK_ENABLE();
  
		StepperMotorBoardHandle_t *StepperMotorBoardHandle;
		MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;
	
		uint8_t board1 = EXPBRD_ID(0);
		uint8_t board2 = EXPBRD_ID(1);
		uint8_t board3 = EXPBRD_ID(2);

		MotorParameterDataGlobal = GetMotorParameterInitData();
  
		for (uint8_t id = 0; id < EXPBRD_MOUNTED_NR; id++)
		{
			StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(id));
			MotorParameterDataSingle = MotorParameterDataGlobal+(id*L6470DAISYCHAINSIZE);
			StepperMotorBoardHandle->Config(MotorParameterDataSingle);
		}

    float Vector[3] = {finalPoint[0] - currentPoint[0], finalPoint[1] - currentPoint[1], finalPoint[2] - currentPoint[2]};
    float magnitude = pow(pow(Vector[0],2) + pow(Vector[1],2) + pow(Vector[2],2), 0.5);
    int iterations = magnitude/70;
    float directionVector[3] = {Vector[0]/iterations, Vector[1]/iterations, Vector[2]/iterations};
		float prevPoint[3];
		float* points[100];
		
    while(iterations){

        for(int i = 0; i < 3; i++){
						prevPoint[i] = currentPoint[i];
            currentPoint[i] = currentPoint[i] + directionVector[i];
        }
				
				MoveNextPoint2(currentAngles, prevPoint, currentPoint);
				
				iterations--;
	}
}
/*
// linear path movement with vector algebra equations			
void LinearMoveNextPoint(float* currentPoint, float* finalPoint, float* currentAngles){
			
		//NUCLEO_Board_Init();																				// NUCLEO board initialization
		HAL_Init();																									// 
		//SystemClock_Config();																				// 
		//BSP_Init();																									// X-NUCLEO-IHM02A1 initialization
	
		//HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
		//HAL_NVIC_SetPriority(SysTick_IRQn, 0, 1);
		
																				// Initialization of Serial communication with UART 6 pins
	
		//ready_message, "42,34,6");

		__HAL_RCC_GPIOA_CLK_ENABLE();
  
		StepperMotorBoardHandle_t *StepperMotorBoardHandle;
		MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;
	
		uint8_t board1 = EXPBRD_ID(0);
		uint8_t board2 = EXPBRD_ID(1);
		uint8_t board3 = EXPBRD_ID(2);

		MotorParameterDataGlobal = GetMotorParameterInitData();
  
		for (uint8_t id = 0; id < EXPBRD_MOUNTED_NR; id++)
		{
			StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(id));
			MotorParameterDataSingle = MotorParameterDataGlobal+(id*L6470DAISYCHAINSIZE);
			StepperMotorBoardHandle->Config(MotorParameterDataSingle);
		}

    float Vector[3] = {finalPoint[0] - currentPoint[0], finalPoint[1] - currentPoint[1], finalPoint[2] - currentPoint[2]};
    float magnitude = pow(pow(Vector[0],2) + pow(Vector[1],2) + pow(Vector[2],2), 0.5);
    int iterations = magnitude/70;
    float directionVector[3] = {Vector[0]/iterations, Vector[1]/iterations, Vector[2]/iterations};
		float prevPoint[3];
    while(iterations){
			
				float angle01 = 0, angle12 = 0, angle23 = 0, angle34 = 0, angle45 = 0;

				float d_angle01 = 0, d_angle12 = 0, d_angle23 = 0, d_angle34 = 0, d_angle45 = 0;
	
				float c_angle01 = currentAngles[0], c_angle12 = currentAngles[1], c_angle23 = currentAngles[2], c_angle34 = currentAngles[3], c_angle45 = currentAngles[4];
	
				//float px = currentPoint[0], py = currentPoint[1], pz = currentPoint[2];

				float h_12 = 302, h_23 = 199;

				float ux = 0, uy = 0, uz = -1;

				float vx = 0, vy = 1, vz = 0;

				float wx = 1, wy = 0, wz = 0;

				float asin_num_frac1 = 0, asin_num_frac2 = 0, asin_num_frac3 = 0, asin_denom_frac1 = 0, asin_denom_frac2 = 0;

        for(int i = 0; i < 3; i++){
						prevPoint[i] = currentPoint[i];
            currentPoint[i] = currentPoint[i] + directionVector[i];
        }
				
				float px = currentPoint[0], py = currentPoint[1], pz = currentPoint[2];
				
				//-------------------------------------------------- ANGLE CALCULATIONS -------------------------------------------------
		
				//base
				angle01 = atan2(py,px);

				//wrist rotation
				angle45 = -atan2(vz,uz);

				//humerus
    
				angle12 =  atan2(pz,pow(pow(px,2)+pow(py,2),0.5)) + acos((pow(h_23,2) - pow(h_12,2) - pow(px,2) - pow(py,2) - pow(pz,2))/(-2*h_12*(pow(pow(px,2) + pow(py,2) + pow(pz,2),0.5))));

				float initial_mag = pow(pow(prevPoint[0],2) + pow(prevPoint[1],2) + pow(prevPoint[2],2),0.5), final_mag = pow(pow(currentPoint[0],2) + pow(currentPoint[1],2) + pow(currentPoint[2],2),0.5);
		
				angle23 = acos((pow(px,2)+pow(py,2)+pow(pz,2)-pow(h_12,2)-pow(h_23,2))/(-2*h_12*h_23));
		
				//elbow
				if(final_mag > initial_mag){
					if(-1*(M_PI-angle23) < currentAngles[2])
						angle23 = M_PI - acos((pow(px,2)+pow(py,2)+pow(pz,2)-pow(h_12,2)-pow(h_23,2))/(-2*h_12*h_23));
				}
		
				//wrist
				angle34 = -atan2(sin(angle01)*wy*cos(angle01) - pow(sin(angle01),2)*wx + wx, cos(angle01)*wz) - angle23 - angle12;
	
				//--------------------------------------------------------D_ANGLE---------------------------------------------------------
		
				d_angle01 = angle01 - c_angle01;
				d_angle12 = angle12 - c_angle12;
				d_angle23 = -1*(M_PI - angle23) + d_angle12 - c_angle23;
				d_angle34 = 0; //angle34 - c_angle34;
				d_angle45 = angle45 - c_angle45;
		
				//---------------------------------------------------- ANGLE MOVEMENTS ---------------------------------------------------
				/*
				//base
				StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(0)); 
				StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(0));
	
				if(d_angle01 > 0)
				StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_BASE*d_angle01);
	
				else
				StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_BASE*d_angle01);
		
				c_angle01 = angle01;
				
				//shoulder
				StepperMotorBoardHandle->Command->HardStop(EXPBRD_ID(0), L6470_ID(1));																								
				//StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(1));
	
				if(d_angle12 > 0){																						
					StepperMotorBoardHandle->Command->Move(EXPBRD_ID(0), L6470_ID(1), L6470_DIR_FWD_ID, STEPS_PER_RAD_HUM*d_angle12);
				}
		
				else{																							
					StepperMotorBoardHandle->Command->Move(EXPBRD_ID(0), L6470_ID(1), L6470_DIR_REV_ID, STEPS_PER_RAD_HUM*d_angle12);
				}
		
				c_angle12 = angle12;
		
	
				//elbow
				StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(0));
				//StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(0));
	
				if(d_angle23 > 0){
					StepperMotorBoardHandle->Command->Move(board2, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_ULN*d_angle23); 
				}
		
				else{
					StepperMotorBoardHandle->Command->Move(board2, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_ULN*d_angle23); 
				}
		
				while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0));
		
				c_angle23 = angle23;
		
				/*
				//wrist
				StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(1));
				StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(1));
	
				if(d_angle34 > 0)
					StepperMotorBoardHandle->Command->Move(board2, L6470_ID(1), L6470_DIR_REV_ID, STEPS_PER_RAD_WRIST*d_angle34); 
	
				else	
					StepperMotorBoardHandle->Command->Move(board2, L6470_ID(1), L6470_DIR_FWD_ID, STEPS_PER_RAD_WRIST*d_angle34); 
	
				c_angle34 = angle34;
				*/
				/*
				//wrist rotation
				StepperMotorBoardHandle->Command->SoftStop(board3, L6470_ID(0));
				StepperMotorBoardHandle->Command->ResetPos(board3, L6470_ID(0));
	
				if(d_angle34 > 0)
					StepperMotorBoardHandle->Command->Move(board3, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_WRIST*d_angle45); 
	
				else	
					StepperMotorBoardHandle->Command->Move(board3, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_WRIST*d_angle45); 
	
				//while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(1), BUSY_ID)==0);
	
				c_angle45 = angle45;
				
	
				currentAngles[0] = angle01;
				currentAngles[1] = angle12;
				currentAngles[2] = -1*(M_PI - angle23);
				currentAngles[3] = angle34;
				currentAngles[4] = angle45;
				
				iterations--;
		}
}
*/
char which_vector(int k){   
   switch(k){   
        case 0:   
            return 'p';   
        case 1:   
            return 'u';   
        case 2:   
            return 'v';   
        case 3:   
            return 'w';   
    }   
    return ' '; 
}   

void TrajectoryPlanning(float* currentPoint, float* currentAngles){
	
		 HAL_Init();		
	
		 MX_USART2_UART_Init();	
	
		 int restart = 0, prev_point = 0, size = 5, rand = 1;  
		 char string[50], input[13], input2[13];
		 char* rest, *rest2;
		 float* nextPoint, a[5][4][3];	//default 5 points for now
		 
		 for(int i=0; i<5; i++){
			 for(int j=0; j<4; j++){
				 for(int k=0; k<3; k++){
					 a[i][j][k] = 1111;
				 }
			 }
		 }
	
    do{   
        restart = 0; 
				
        for(int i=0; i<5; i++){
					for(int j=0; j<4; j++){
						for(int k=0; k<3; k++){
								a[i][j][k] = 1111;
						}
					}
				}
				
				USART_Transmit(&huart2, "For each point, enter the position and end effector vectors in the following format.\n");
				USART_Transmit(&huart2, "For each point, enter the position and end effector vectors in the following format.\n");
				USART_Transmit(&huart2, "px py px\n");
				USART_Transmit(&huart2, "ux uy uz\n");
				USART_Transmit(&huart2, "vx vy vz\n"); 
				USART_Transmit(&huart2, "wx wy wz\n");
				USART_Transmit(&huart2, "The format must be: xxx(space)yyy(space)zzz(enter). If a number is less than 3 digits, fill remaining digits with spaces.");
				USART_Transmit(&huart2, "To restart your current vector, enter 777 for x,y,or z.\n");
				USART_Transmit(&huart2, "To restart your previous vector, enter 888 for x,y,or z, repeatedly to move back several vectors.\n");
				USART_Transmit(&huart2, "To restart your entire entry, enter 999 for x,y,or z.\n");
				
        for (int i = 0; i < size; i++){   
						sprintf(string, "Point %d:\n", i+1);	
						SerialPuts2(string);
            for (int k = 0; k < 4; k++){   
                if(prev_point == 1){   
                    prev_point = 0;   
                    k = 3;   
                }
								rest = input;
								sprintf(string, "%c: ", which_vector(k));
								SerialPuts2(string);
								HAL_UART_Receive(&huart2, (uint8_t*)input, 13, HAL_MAX_DELAY);
								USART_Transmit(&huart2, input);
                for (int j = 0; j < 3; j++){
										a[i][k][j] = atoi(strtok_r(rest, " ", &rest));
                    if(a[i][k][j] == 777){   
                        sprintf(string, "%s\n", "You have entered 777. Please re-enter the current vector: \n");
												SerialPuts2(string);
                        k--;   
												break;
                    }   

                    else if(a[i][k][j] == 888){   
                        if(k == 0){   
                            if(i == 0){   
                                sprintf(string, "No previous vectors. Please enter the first vector for Point 1:\n");
																SerialPuts2(string);  
                                k--;   
                                break;   
                            }   

                            else{   
                                sprintf(string, "You have entered 888. Please re-enter the last vector: \n");
																SerialPuts2(string);
                                prev_point = 1;   
                                i=i-2;   
                                break;   
                            }   
                        }   

                        else if(k > 0){   
                            sprintf(string, "You have entered 888. Please re-enter the last vector (%c for Point %d) :\n", which_vector(k-1), i+1);
														SerialPuts2(string);
                            k=k-2;   
                            break;   
                        }   
                    }   

                    else if(a[i][k][j] == 999){   
                        restart = 1;   
                        sprintf(string, "You have entered 999. Please restart.\n");
												SerialPuts2(string);
												break;   
                    }
                }   
 
                if(restart == 1)   
                    break;   

                else if(prev_point == 1)   
										break;   
            }   
            if(restart == 1){   
                break;   
            }   
        }   
        SerialPutc2('\n');
				SerialPutc2('\n');
		}while(restart == 1);
		
		for(int i = 0; i < 5; i++){
			for(int k = 0; k < 3; k++){
				nextPoint[k] = a[i][0][k];
				sprintf(string, "%f\n", nextPoint[k]);
				MoveNextPoint2(currentAngles, currentPoint, nextPoint);
			}
		}
}

int main(void){

		//initialization
		NUCLEO_Board_Init();																				
		HAL_Init();				
		SystemClock_Config();
		BSP_Init();																						

		MX_USART6_UART_Init();																		
		MX_USART2_UART_Init();	
	
		uint8_t id;
	
		uint8_t hold_message[4];

		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
			
		StepperMotorBoardHandle_t *StepperMotorBoardHandle;
		MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;
  
		uint8_t board1 = EXPBRD_ID(0);
		uint8_t board2 = EXPBRD_ID(1);
		uint8_t board3 = EXPBRD_ID(2);
  
		/* Get the parameters for the motor connected with the 1st stepper motor driver of the 1st stepper motor expansion board */
		MotorParameterDataGlobal = GetMotorParameterInitData();
			
		for (id = 0; id < EXPBRD_MOUNTED_NR; id++)
		{
			StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(id));
			MotorParameterDataSingle = MotorParameterDataGlobal+(id*L6470DAISYCHAINSIZE);
			StepperMotorBoardHandle->Config(MotorParameterDataSingle);
		}
		// end initialization
		
		float currentAngles[5] = {0}, currentPoint[3] = {0}, nextPoint[3] = {0};
		char string[50], error[15] = "Invalid input";
		sprintf(string, "%s", "hi");
		
		Home_Arm(currentAngles, currentPoint);
		while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0));
		
		TrajectoryPlanning(currentPoint, currentAngles);

		/*
		nextPoint[0] = 300;
		nextPoint[1] = 200;
		nextPoint[2] = 150;
		
		
		LinearMove(currentPoint, nextPoint, currentAngles);
		*/
		/*
		while(1) {
			
			HAL_UART_Receive(&huart2, hold_message, 4, HAL_MAX_DELAY);
			int c1 = hold_message[0]-48;
			int c2 = hold_message[1]-48;
			
			sprintf(string, "Pick up: conveyor %d \n", c1);
			SerialPuts2(string);
			sprintf(string, "Drop off: conveyor %d \n", c2);
			SerialPuts2(string);
			
			switch(c1){
				case 1:
					nextPoint[0] = 300;
					nextPoint[1] = 300;
					nextPoint[2] = 50;
					break;
				case 2:
					nextPoint[0] = -300;
					nextPoint[1] = 300;
					nextPoint[2] = 50;
					break;
				case 3:
					nextPoint[0] = -300;
					nextPoint[1] = -300;
					nextPoint[2] = 50;
					break;
				case 4:
					nextPoint[0] = 300;
					nextPoint[1] = -300;
					nextPoint[2] = 50;
					break;
				default:
					SerialPuts2(error);
					break;
		}
			
		MoveNextPoint2(currentAngles, currentPoint, nextPoint);
		HAL_Delay(300);
		
		nextPoint[0] = currentPoint[0]/1.5;
		nextPoint[1] = currentPoint[1]/1.5;
		nextPoint[2] = 150;
	
		MoveNextPoint2(currentAngles, currentPoint, nextPoint);
		
		switch(c2){
				case 1:
					nextPoint[0] = 300;
					nextPoint[1] = 300;
					nextPoint[2] = 50;
					break;
				case 2:
					nextPoint[0] = -300;
					nextPoint[1] = 300;
					nextPoint[2] = 50;
					break;
				case 3:
					nextPoint[0] = -300;
					nextPoint[1] = -300;
					nextPoint[2] = 50;
					break;
				case 4:
					nextPoint[0] = 300;
					nextPoint[1] = -300;
					nextPoint[2] = 50;
					break;
				default:
					SerialPuts2(error);
					break;
		}
		
		MoveNextPoint2(currentAngles, currentPoint, nextPoint);
		HAL_Delay(300);
		
		nextPoint[0] = currentPoint[0]/1.5;
		nextPoint[1] = currentPoint[1]/1.5;
		nextPoint[2] = 150;
	
		MoveNextPoint2(currentAngles, currentPoint, nextPoint);
	}
	*/
}

//------------------------------------------------------------------

static void Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while (1)
  {
  }
}

void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}


uint16_t Read_ADC(void)
{
  HAL_ADC_Start(&HADC);
  HAL_ADC_PollForConversion(&HADC, 100);
  
  return HAL_ADC_GetValue(&HADC);
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART6_UART_Init(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
}
