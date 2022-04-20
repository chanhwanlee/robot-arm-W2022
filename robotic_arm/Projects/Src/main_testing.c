#include "xnucleoihm02a1.h"
#include "example.h"

#define HOME_SPEED 30000 // conveyor speed
#define M_PI 3.14159265358979323846

#define STEPS_PER_RAD_BASE 38003.0 	// 5 2/11 gearbox from 1/128th microstepping motor, 10/18 belt reduction
#define STEPS_PER_RAD_HUM 111911.0   // 13 212/289 gearbox from 1/128th microstepping motor, 10/20 belt reduction
#define STEPS_PER_RAD_ULN 111911.0  // same, for now
#define STEPS_PER_RAD_WRIST 111911.0  // same, for now


uint16_t CurrBoard = 0; // needed for compiling

UART_HandleTypeDef huart1; // needed for compiling
UART_HandleTypeDef huart2; // needed for compiling

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int main(void) {
	// ---------------------- Configuration and Initialization --------------------------------
	NUCLEO_Board_Init();
	HAL_Init();
	SystemClock_Config();
  BSP_Init();
	HAL_Delay(1000);
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	StepperMotorBoardHandle_t *StepperMotorBoardHandle;
  MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;
  
	uint8_t board1 = EXPBRD_ID(0);
	uint8_t board2 = EXPBRD_ID(1);
	uint8_t board3 = EXPBRD_ID(2);
	
	int d_angle[5] = {0}, next_angle[5] = {0}, curr_angle[5] = {0};
	int pi[3] = {0}, pf[3] = {0}, u[3] = {0, 0, -1}, v[3] = {-1, 0, 0}, w[3] = {0, 1, 0};
	int a12_atan_num = 0, a12_atan_denom = 0, a12_asin_num = 0, a12_asin_denom = 0, h_12 = 1, h_23 = 1;
		
	// wait for coordinates from PLCs; take in coordinates in global[2]
	// object coordinates:  xi = pi[0] ; yi = pi[1] ; zi = pi[2]
	//											xf = pf[0] ; yf = pf[1] ; zf = pf[2]
	// gripper orientation: u = [0, 0, -1] ; v = [-1, 0, 0] ; w = [0, 1, 0]

  MotorParameterDataGlobal = GetMotorParameterInitData();
  
  for (uint8_t id = 0; id < EXPBRD_MOUNTED_NR; id++)
  {
    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(id));
    MotorParameterDataSingle = MotorParameterDataGlobal+(id*L6470DAISYCHAINSIZE);
    StepperMotorBoardHandle->Config(MotorParameterDataSingle);
  }
	
	Home_Arm();
	
	curr_angle[0] = M_PI/2;						// base
	curr_angle[1] = M_PI/2;						// humerus
	curr_angle[2] = M_PI/2;						// elbow
	curr_angle[3] = 0;								// wrist
	curr_angle[4] = 0;								// wrist rotation
	
	/*
	// ----------------------- Commands --------------------------------------------------------
	StepperMotorBoardHandle->Command->SoftStop(board1, L6470_ID(0)); 
	StepperMotorBoardHandle->Command->SoftStop(board1, L6470_ID(1)); 
	StepperMotorBoardHandle->Command->SoftStop(board2, L6470_ID(0)); 
	StepperMotorBoardHandle->Command->SoftStop(board2, L6470_ID(1));
	StepperMotorBoardHandle->Command->SoftStop(board3, L6470_ID(0));

	StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(0));
	StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(1));
	StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(0));
	StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(1));
	StepperMotorBoardHandle->Command->ResetPos(board3, L6470_ID(0));
	
	TBD:
					- link lengths
					- initial angles 
					- wire wrist rotation, gripper
					- servo wiring, control
					- take in coordinates from plcs
					- loop
	*/
		
	//take in coordinates from plcs (pi[2], pf[2])
	
	//----------------------------------------------- PICK-UP ANGLE CALCULATIONS--------------------------------------------
	//base
	next_angle[0] = atan2(pi[1],pi[0]);
	
	//humerus
	a12_atan_num = sin(next_angle[0]*pi[1]*cos(next_angle[0]) - (pow(sin(next_angle[0]), 2)*pi[0]) + pi[0]);
	a12_atan_denom = cos(next_angle[0])*pi[2];
	
	a12_asin_num = (pow((sin(next_angle[0])*pi[1]*cos[0] - (pow(sin(next_angle[0]), 2)*pi[0]) + pi[0]), 2)/(pow(cos(next_angle[0]), 2)*pow(h_23, 2))) + (pow(pi[2], 2)/pow(h_23, 2)) - (pow(h_12, 2)/pow(h_23, 2));
	a12_asin_denom = pow((((4*pow((sin(next_angle[0])*pi[1]*cos(next_angle[0]) - pow(sin(next_angle[0]), 2)*pi[0] + pi[0]), 2)*pow(h_12, 2))/(pow(cos(next_angle[0]), 2)*pow(h_23, 4))) + (4*pow(pi[2], 2)*pow(h_12, 2)/pow(h_23,4))), 0.5);
	
	next_angle[1] = -atan2(a12_atan_num/a12_atan_denom) + asin(a12_asin_num/a12_asin_denom);
	
	//elbow
	next_angle[2] = -atan2(((sin(next_angle[0])*h_12 - pi[2])*cos(next_angle[0]))/(sin(next_angle[0])*pi[1]*cos(next_angle[0]) - cos(next_angle[0])*cos(next_angle[1])*h_12 - pow(sin(next_angle[0]),2)*pi[0] + pi[0])) - next_angle[1];
	
	//wrist
	next_angle[3] = -atan2(((sin(next_angle[0])*w[1]*cos(next_angle[0])-pow(sin[0],2)*w[0] + w[0]))/(cos(next_angle[0]*w[2]))) - next_angle[2] - next_angle[1];
	
	//wrist rotation (always 0 for now)
	next_angle[4] = -atan2(v[2]/u[2]);
	
	//--------------------------------------------------- d_angle --------------------------------------------------
	
	for(int i = 0; i < 5; i++){
		d_angle[i] = next_angle[i] - curr_angle[i];
	}
	
	//------------------------------------------------ PICK-UP ANGLE MOVEMENTS ----------------------------------------------
	
	//base
	if(d_angle[0] > 0) {
			StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_BASE*d_angle[0]);
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board1, L6470_ID(0), BUSY_ID)==0);
	}
	
	else{
			StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_BASE*abs(d_angle[0]));
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board1, L6470_ID(0), BUSY_ID)==0);
	}
	
	curr_angle[0] = next_angle[0];
	
	//humerus
	if(d_angle[1] > 0){
		StepperMotorBoardHandle->Command->Move(board1, L6470_ID(1), L6470_DIR_FWD_ID, STEPS_PER_RAD_HUM*d_angle[1]);
		StepperMotorBoardHandle->Command->Run(board1, L6470_ID(1), L6470_DIR_FWD_ID, 0.15*HOME_SPEED);	
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board1, L6470_ID(1), BUSY_ID)==0);
	}
	
	else{
		StepperMotorBoardHandle->Command->Move(board1, L6470_ID(1), L6470_DIR_REV_ID, STEPS_PER_RAD_HUM*abs(d_angle[1]));
		StepperMotorBoardHandle->Command->Run(board1, L6470_ID(1), L6470_DIR_REV_ID, 0.15*HOME_SPEED);	
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board1, L6470_ID(1), BUSY_ID)==0);
	}
	
	curr_angle[1] = next_angle[1];
	
	// ulna
	if(d_angle[2] > 0){
		StepperMotorBoardHandle->Command->Move(board2, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_ULN*d_angle[2]); 
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(0), BUSY_ID)==0);
	}
	
	else{
		StepperMotorBoardHandle->Command->Move(board2, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_ULN*abs(d_angle[2])); 
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(0), BUSY_ID)==0);
	}
	
	curr_angle[2] = next_angle[2];
	
	// wrist
	if(d_angle[3] > 0){
		StepperMotorBoardHandle->Command->Move(board2, L6470_ID(1), L6470_DIR_FWD_ID, STEPS_PER_RAD_WRIST*d_angle[3]); 
		StepperMotorBoardHandle->Command->Run(board2, L6470_ID(1), L6470_DIR_FWD_ID, 0.05*HOME_SPEED);
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(1), BUSY_ID)==0);
	}
	
	else{
		StepperMotorBoardHandle->Command->Move(board2, L6470_ID(1), L6470_DIR_REV_ID, STEPS_PER_RAD_WRIST*abs(d_angle[3])); 
		StepperMotorBoardHandle->Command->Run(board2, L6470_ID(1), L6470_DIR_FWD_ID, 0.05*HOME_SPEED);
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(1), BUSY_ID)==0);
	}
	
	curr_angle[3] = next_angle[3];
	
	// wrist rotation
	/*
	if(d_angle[4] > 0){
		StepperMotorBoardHandle->Command->Run(board3, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_WRIST_ROT*d_angle[4]);
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(1), BUSY_ID)==0);
	}
	
	else{
		StepperMotorBoardHandle->Command->Run(board3, L6470_ID(0), L6470_DIR_REV_ID, STEPS_PER_RAD_WRIST_ROT*d_angle[4]);
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(1), BUSY_ID)==0);
	}
	
	curr_angle[4] = next_angle[4];
	
	// open gripper
	
	*/
	
	//------------------------------------------------ DROP-OFF ANGLE MOVEMENTS ----------------------------------------------
	next_angle[0] = atan2(pf[1]/pf[0]);
	
	//humerus
	a12_atan_num = sin(next_angle[0]*pf[1]*cos(next_angle[0]) - (pow(sin(next_angle[0]), 2)*pf[0]) + pf[0]);
	a12_atan_denom = cos(next_angle[0])*pf[2];
	
	a12_asin_num = (pow((sin(next_angle[0])*pf[1]*cos[0] - pow(sin(next_angle[0]), 2)*pf[0] + pf[0]), 2)/(pow(cos(next_angle[0]), 2)*pow(h_23, 2))) + (pow(pf[2], 2)/pow(h_23, 2)) - (pow(h_12, 2)/pow(h_23, 2));
	a12_asin_denom = pow((((4*pow((sin(next_angle[0])*pf[1]*cos(next_angle[0]) - pow(sin(next_angle[0]), 2)*pf[0] + pf[0]), 2)*pow(h_12, 2))/(pow(cos(next_angle[0]), 2)*pow(h_23, 4))) + (4*pow(pi[2], 2)*pow(h_12, 2)/pow(h_23,4))), 0.5);
	
	next_angle[1] = -atan2(a12_atan_num/a12_atan_denom) + asin(a12_asin_num/a12_asin_denom);
	
	//elbow
	next_angle[2] = -atan2(((sin(next_angle[0])*h_12 - pf[2])*cos(next_angle[0]))/(sin(next_angle[0])*pf[1]*cos(next_angle[0]) - cos(next_angle[0])*cos(next_angle[1])*h_12 - pow(sin(next_angle[0]),2)*pf[0] + pf[0])) - next_angle[1];
	
	//wrist
	next_angle[3] = -atan2(((sin(next_angle[0])*w[1]*cos(next_angle[0])-pow(sin[0],2)*w[0] + w[0]))/(cos(next_angle[0]*w[2]))) - next_angle[2] - next_angle[1];
	
	//wrist rotation (always 0 for now)
	next_angle[4] = -atan2(v[2]/u[2]);
	
	
}

	/****************************************************************************************************************
	// base
	//StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_BASE*M_PI);
	//while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board1, L6470_ID(0), BUSY_ID)==0);
	
	// humerus
	//StepperMotorBoardHandle->Command->Move(board1, L6470_ID(1), L6470_DIR_FWD_ID, STEPS_PER_RAD_HUM*M_PI/4);
	//StepperMotorBoardHandle->Command->Run(board1, L6470_ID(1), L6470_DIR_FWD_ID, 0.15*HOME_SPEED);	
	//while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board1, L6470_ID(1), BUSY_ID)==0);

	// ulna
	//StepperMotorBoardHandle->Command->Move(board2, L6470_ID(0), L6470_DIR_FWD_ID, STEPS_PER_RAD_ULN*M_PI/2); 
	//while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(0), BUSY_ID)==0);
	
	// wrist
	//StepperMotorBoardHandle->Command->Move(board2, L6470_ID(1), L6470_DIR_FWD_ID, STEPS_PER_RAD_WRIST*M_PI/4); 
	//StepperMotorBoardHandle->Command->Run(board2, L6470_ID(1), L6470_DIR_FWD_ID, 0.05*HOME_SPEED);
	//while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(1), BUSY_ID)==0);
	
	// wrist rotation
	//StepperMotorBoardHandle->Command->Run(board3, L6470_ID(0), L6470_DIR_FWD_ID, 0.05*HOME_SPEED);
	//while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(1), BUSY_ID)==0);
	
}


// Assert required to compile without errors
#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif
