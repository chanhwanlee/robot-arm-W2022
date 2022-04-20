
	#include "example.h"
	
	#define  PERIOD_VALUE       (uint32_t)(666 - 1)  /* Period Value  */
	#define  PULSE1_VALUE       (uint32_t)(PERIOD_VALUE/2)        /* Capture Compare 1 Value  */
	#define  PULSE2_VALUE       (uint32_t)(PERIOD_VALUE*37.5/100) /* Capture Compare 2 Value  */
	#define  PULSE3_VALUE       (uint32_t)(PERIOD_VALUE/4)        /* Capture Compare 3 Value  */
	#define  PULSE4_VALUE       (uint32_t)(PERIOD_VALUE*12.5/100)
	
	#define z_offset 187 //z offset in mm
	#define Ulna 190 // [ mm ]
	#define Humerus 290 // [ mm ]
	#define Wrist_Link = 300 // [ mm ]
	
	
	#define M_PI 3.14159265358979323846
	
	#define HOME_HUMERUS 1.45 // HOME_HUM and LIM_HUM must sum to Pi/2                   //limit switch angle = 0.16rad
	#define HOME_TEST 0.4
	#define HOME_ULNA 0.349			
	#define HOME_WRIST 0.60
	#define HOME_BASE M_PI/2
	#define HOME_WRIST_ROT M_PI/2
	
	#define LIM_HUM 0.14 // HOME_HUM and LIM_HUM must sum to Pi/2
	#define LIM_ULN 0.0
	#define LIM_WST 0.1
	
	#define STEPS_PER_RAD_BASE 38003.0 	// 5 2/11 gearbox from 1/128th microstepping motor, 10/18 belt reduction
	#define STEPS_PER_RAD_HUM 111911.0   // 13 212/289 gearbox from 1/128th microstepping motor, 10/20 belt reduction
	#define STEPS_PER_RAD_ULN 111911.0  // same, for now
	#define STEPS_PER_RAD_Wrist 111911.0  // same, for now
	#define STEPS_PER_RAD_Wrist_Rot (128*200/(2*M_PI)) // 200 steps/2pi rad, 1/128th microstepping motor

	/*
	#define MAX_ULNA_MOTOR_ANGLE 180 //degrees
	#define MAX_HUMERUS_MOTOR_ANGLE 90 //degrees
	#define MIN_ULAN_MOTOR_ANGLE 90 //degrees
	#define MIN_HUMERUS_MOTOR_ANGLE 20 //degrees
	*/
	
	#define HOME_SPEED 30000
	
	#define STEPS_PER_ROTATION 351488 
	#define GRIPPER_STEPS_PER_ROTATION 25600
	
	#define ALPHA_CORRECTION_RATIO 1.50
	#define BETA_CORRECTION_RATIO 1.50
	#define GAMMA_CORRECTION_RATIO 1.50
	
	#define GAMMA_STARTING_ANGLE 0.5307787
	
	/*
	IMPORTANT
	Do not use the following GPIO pins, they are reserved for the SPI communication of the Motor Boards and UART communication
	GPIOA pins 4 and 10
	GPIOB pins 4, 5 and 6
	GPIOC pins 0, 1, 6 and 7
	
	Unavailable pins
	A:  0,1,4,5,6,8,10
	B: 	4,5,6
	C: 	0,1,6,7
	*/
	
	#define HUMERUS_MAX_PIN GPIO_PIN_8
	#define ULNA_MIN_PIN GPIO_PIN_6 
	#define WRIST_MID_PIN GPIO_PIN_1 // A1 on Board
	#define BASE_HOME GPIO_PIN_0 
	#define WRIST_ROT_PIN GPIO_PIN_5
	
	#define GRIP_A GPIO_PIN_7 // D5 on Board
	#define GRIP_B GPIO_PIN_9 // D4 on Board
	
	#define OPEN 1
	#define CLOSE 0

	eL6470_DirId_t find_Dir_beta(float steps);
	eL6470_DirId_t find_Dir_gama(float steps);
	void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);
	void user_pwm_setvalue(uint16_t value);
	static void SystemClock_Config(void);
	int gripper_angle_to_steps(float angle);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_HandleTypeDef htim4;
	static UART_HandleTypeDef huart2;

void Home_Arm(float* currentAngles, float* currentPoint)
{
	uint8_t id;
	//HAL_Init();
	
	//added
	NUCLEO_Board_Init();																				// NUCLEO board initialization
	HAL_Init();																									// 
	//SystemClock_Config();																				// 
	BSP_Init();
	//
	uint8_t buffer[10];
	uint8_t data[6];
	
	//initilization of GPIO pins used for Limit switches 	
	__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin   			= HUMERUS_MAX_PIN;
	GPIO_InitStructure.Mode  			= GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull 		 	= GPIO_NOPULL;
	GPIO_InitStructure.Speed 			= GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin   			= ULNA_MIN_PIN;
	GPIO_InitStructure.Mode  			= GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull  			= GPIO_NOPULL;
	GPIO_InitStructure.Speed 			= GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin   			= WRIST_MID_PIN;
	GPIO_InitStructure.Mode  			= GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull  			= GPIO_NOPULL;
	GPIO_InitStructure.Speed 			= GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin   			= BASE_HOME;
	GPIO_InitStructure.Mode  			= GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull  			= GPIO_PULLDOWN;
	GPIO_InitStructure.Speed 			= GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin   			= WRIST_ROT_PIN;
	GPIO_InitStructure.Mode  			= GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull  			= GPIO_NOPULL;
	GPIO_InitStructure.Speed 			= GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*
	// Gripper
		__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin   			= GRIP_A;
	GPIO_InitStructure.Mode  			= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed 			= GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin   			= GRIP_B;
	GPIO_InitStructure.Mode  			= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed 			= GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	//--
	*/
	
  StepperMotorBoardHandle_t *StepperMotorBoardHandle;
  MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;
  
	uint8_t board1 = EXPBRD_ID(0);
	uint8_t board2 = EXPBRD_ID(1);
	uint8_t board3 = EXPBRD_ID(2);
	
  /* Setup each X-NUCLEO-IHM02A1 Expansion Board ******************************/
  
  /* Get the parameters for the motor connected with the 1st stepper motor driver of the 1st stepper motor expansion board */
  MotorParameterDataGlobal = GetMotorParameterInitData();
  
  for (id = 0; id < EXPBRD_MOUNTED_NR; id++)
  {
    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(id));
    MotorParameterDataSingle = MotorParameterDataGlobal+(id*L6470DAISYCHAINSIZE);
    StepperMotorBoardHandle->Config(MotorParameterDataSingle);
  }
	
	//HOME BASE
	StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(0));
	if(currentAngles[0] > -M_PI/2 && currentAngles[0] < M_PI/2){
		StepperMotorBoardHandle->Command->Run(board1, L6470_ID(0), L6470_DIR_FWD_ID, 0.125*HOME_SPEED); 								// Run motor in reverse direction
		while(!HAL_GPIO_ReadPin(GPIOA, BASE_HOME)); 																																		// Wait until limit switch is pushed
		StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(0)); 																								// Stop motor
		StepperMotorBoardHandle->Command->Run(board1, L6470_ID(0), L6470_DIR_REV_ID, 0.125*HOME_SPEED); 								// Run motor in forward direction
		while(HAL_GPIO_ReadPin(GPIOA, BASE_HOME)); 																																			// Wait until limit switch is released
		StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(0)); 																								// Stop motor
		StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(0)); 																								// ?
		StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_BASE*HOME_BASE); 		// Move base to preset position
	}
	else{
		StepperMotorBoardHandle->Command->Run(board1, L6470_ID(0), L6470_DIR_REV_ID, 0.125*HOME_SPEED); 								// Run motor in reverse direction
		while(!HAL_GPIO_ReadPin(GPIOA, BASE_HOME)); 																																		// Wait until limit switch is pushed
		StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(0)); 																								// Stop motor
		StepperMotorBoardHandle->Command->Run(board1, L6470_ID(0), L6470_DIR_FWD_ID, 0.125*HOME_SPEED); 								// Run motor in forward direction
		while(HAL_GPIO_ReadPin(GPIOA, BASE_HOME)); 																																			// Wait until limit switch is released
		StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(0)); 																								// Stop motor
		StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(0)); 																								// ?
		StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_BASE*HOME_BASE); 		// Move base to preset position
	}

	//HOME HUMERUS
	StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(1));																								// ?																		// t.o.?
	StepperMotorBoardHandle->Command->Run(board1, L6470_ID(1), L6470_DIR_REV_ID, HOME_SPEED);												// Run motor in reverse direction
	while(!HAL_GPIO_ReadPin(GPIOA, HUMERUS_MAX_PIN));																																// Wait until limit switch is pushed
	StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(1));
	StepperMotorBoardHandle->Command->Run(board1, L6470_ID(1), L6470_DIR_FWD_ID, HOME_SPEED);												// Run motor in forward direction
	while(HAL_GPIO_ReadPin(GPIOA, HUMERUS_MAX_PIN));																																// Wait until limit switch is released	// t.o.?
	StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(1));																								// Stop motor														// t.o.?
	StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(1));																								// ? 																		// t.o.?
	StepperMotorBoardHandle->Command->Move(board1, L6470_ID(1), L6470_DIR_FWD_ID, STEPS_PER_RAD_HUM*HOME_HUMERUS);											// Move to preset position	

	//HOME ULNA
	StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(0)); 																								// ? 																		// t.o.?
	StepperMotorBoardHandle->Command->Run(board2, L6470_ID(0), L6470_DIR_REV_ID, HOME_SPEED); 											// Run motor in reverse direction
	while(!HAL_GPIO_ReadPin(GPIOA, ULNA_MIN_PIN)); 																																	// Wait until limit switch is pushed		
	StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(0));																								// Stop motor
	StepperMotorBoardHandle->Command->Run(board2, L6470_ID(0), L6470_DIR_FWD_ID, HOME_SPEED);												// Run motor in forward direction
	while(HAL_GPIO_ReadPin(GPIOA, ULNA_MIN_PIN)); 																																	// Wait until limit switch is released	// t.o.?
	StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(0));																								// Stop motor														// t.o.?
	StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(0));																								// ? 																		// t.o.?
	StepperMotorBoardHandle->Command->Move(board2, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_ULN*HOME_ULNA);			// Move to preset position
	while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0));
	
	/*
	//HOME WRIST ANGLE
	StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(1));																								// ? 																		// t.o.?
	StepperMotorBoardHandle->Command->Run(board2, L6470_ID(1), L6470_DIR_FWD_ID, HOME_SPEED);												// Run motor in forward direction 		
	while(!HAL_GPIO_ReadPin(GPIOA, WRIST_MID_PIN));																															  	// Wait until limit switch is hit
	StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(1));																								// Stop motor
	StepperMotorBoardHandle->Command->Run(board2, L6470_ID(1), L6470_DIR_REV_ID, HOME_SPEED);												// Run motor in reverse direction
	while(HAL_GPIO_ReadPin(GPIOA, WRIST_MID_PIN));																															  	// Wait until limit switch is released 	// t.o.?
	StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(1));																								// Stop motor														// t.o.?
	StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(1));																								// ? 																		// t.o.?
	StepperMotorBoardHandle->Command->Move(board2, L6470_ID(1), L6470_DIR_REV_ID, STEPS_PER_RAD_Wrist*HOME_WRIST);	// Run motor to preset position

	//HOME WRIST ROTATION
	StepperMotorBoardHandle->Command->ResetPos(board3, L6470_ID(0));
	StepperMotorBoardHandle->Command->Run(board3, L6470_ID(0), L6470_DIR_FWD_ID, HOME_SPEED);												
	while(!HAL_GPIO_ReadPin(GPIOA, WRIST_ROT_PIN));
	StepperMotorBoardHandle->Command->HardStop(board3, L6470_ID(0));																								
	StepperMotorBoardHandle->Command->Run(board3, L6470_ID(0), L6470_DIR_REV_ID, HOME_SPEED);
	while(HAL_GPIO_ReadPin(GPIOA, WRIST_ROT_PIN));
	StepperMotorBoardHandle->Command->HardStop(board3, L6470_ID(0));																							
	StepperMotorBoardHandle->Command->ResetPos(board3, L6470_ID(0));																								
	StepperMotorBoardHandle->Command->Move(board3, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_Wrist_Rot*HOME_WRIST_ROT);
	*/
	currentAngles[0] = 0;
	currentAngles[1] = M_PI/2;
	currentAngles[2] = -M_PI/2;
	currentAngles[3] = -M_PI/2;
	currentAngles[4] = 0;
		
	currentPoint[0] = 199;
	currentPoint[1] = 0;
	currentPoint[2] = 302;
}

eL6470_DirId_t find_Dir_beta(float steps){
	if(steps>=0){
		return L6470_DIR_FWD_ID;
	}
	else{
		return L6470_DIR_REV_ID;
	}
}

//currently not working function, needs different not interfering GPIO pins
void Gripper(short grp_st)
{
	
	HAL_Init();
	
	
}
