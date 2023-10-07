/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************** ************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "icm20948.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define Calibrate 1

#define DEBOUNCE_DELAY_MS 200 // Adjust the debounce delay as needed (experiment with different values)

// Global variables
uint8_t buttonState = 0; // Variable to store the current debounced button state (0 = not pressed, 1 = pressed)
uint32_t lastDebounceTime = 0;
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

// *******All Variables Scope(Global) ************/

uint8_t buf[100];

uint8_t flag = 1;
//volatile bool buttonState = false;
volatile uint32_t timedelay = 0 ;
volatile uint32_t mode = 0;
volatile uint8_t pressButton =0;

float pitch = 0, yaw = 0, roll = 0, height = 0;
float pitch2 = 0, yaw2 = 0, roll2 = 0;
float elapsedTime, currentTime = 0, previousTime = 0 , step_count= 0;
float standTime= 0 , sitTime = 0, layTime = 0, walkTime=0;
float acc_angle_x, acc_angle_y, acc_angle_z;
float gcc_angle_x, gcc_angle_y, gcc_angle_z;
int acc_x, acc_y, acc_z, acc_r;
int acc_x2, acc_y2, acc_z2, acc_r2;
int gcc_x, gcc_y, gcc_z;
int gcc_x2, gcc_y2, gcc_z2;
int gyro_x,gyro_y,gyro_z;
float gAngX, gAngY, gAngZ;
float gAngX2, gAngY2, gAngZ2;
int x, y, z;
int x2, y2, z2;
int motion_count = 0, count = 0, lying = 0, stand = 0, sit = 0, walking = 0;
uint32_t dc_x2,dc_y2,dc_z2;
uint32_t dc_x1,dc_y1,dc_z1;

int minVal = 265;
int maxVal = 402;
ICM20948_t ICMData;
ICM20948_t ICMData2;
bool tof, walk, trigger = false, stnd = false, st = false, lay = false, wlk = false , fall = false;
bool fstnd = true, fst = true, flay = true, fwlk = true;
float acc_av_x = 0 , acc_av_y = 0 , acc_av_z = 0; // acceleration average -
float angle_av_x = 0, angle_av_y = 0 , angle_av_z  = 0; // angle Average -
float pitch_av = 0 , roll_av = 0 , yaw_av = 0; // pitch roll yaw Average -


//GLOBALY DECLARED required for Mahony Filter
//Vector to hold parameter
float q[4] = {1.0,0.0,0.0,0.0};

//Free parameters in the Mahony Filter and fusion scheme,
// KP for proportional Feedback , Ki for integral feedback
float Kp =30.0;
float Ki =0.0;


void SystemClock_Config(void);
float average(float average){
	return average/100;
}
long map( long x , long in_min , long in_max, long out_min , long out_max){
	return (x - in_min) * (out_max - out_min)/(in_max - in_min )+ out_min ;
}

/*************************Send Uart ***********************/
void send_uart(char *string){
	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart1, (uint8_t*) string, len, HAL_MAX_DELAY); //Transmit in blocking mode
}

void held_press(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,SET);
	HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,SET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,RESET);
}
/******* Inflation On (Here it is defined as the cushion get inflate)********/
void inflation_On(){
	HAL_Delay(200);
		 // This code to soft start bldc motor
			for(int i=0; i <= 100; i++){
				TIM1->CCR1 = i;
				TIM2->CCR2 = i;
				HAL_Delay(1);
			}
			HAL_Delay(1000);
			//This for loop to turn on bldc motor
			for(int i=100; i <= 120; i++){
				HAL_Delay(20);
				TIM1->CCR1 = i + 6 ;
				TIM2->CCR2 = i + 6 ;
			}
			HAL_Delay(5000);


		//This for rotating MG90s first Servo motor to 90 deg
		TIM1->CCR4 = 140;
	    HAL_Delay(20);
	    TIM1->CCR4 = 175;
	    HAL_Delay(20);
	    TIM1->CCR4 = 250;
	    HAL_Delay(200);
	    //This for rotating MG90s second Servo motor to 90 deg
	    HAL_Delay(200);
	    TIM2->CCR1 = 140;
	    HAL_Delay(20);
	    TIM2->CCR1 = 175;
	    HAL_Delay(20);
	    TIM2->CCR1 = 250;
	    HAL_Delay(200);

	    for(int i=120; i >= 0; i--){
	    	TIM1->CCR1 = i ;
	    	TIM2->CCR2 = i ;
	    	HAL_Delay(5);
	    }
}
/******* Inflation Off ( Here it is defined as the cushion get Deflate)  *******/
void inflation_Off(){

    TIM1->CCR4 = 250;
    HAL_Delay(20);
    TIM1->CCR4 = 175;
    HAL_Delay(20);
    TIM1->CCR4 = 140;
    HAL_Delay(200);

    TIM2->CCR1 = 250;
    HAL_Delay(30);
    TIM2->CCR1 = 175;
    HAL_Delay(20);
    TIM2->CCR1 = 140;
    HAL_Delay(400);
}

/************* Acceleration Noise Removal ******************/

int16_t noise_removal(int16_t raw_accel ){
      float sampleRate = 100.0;
      float cutoffFrequency = 20.0;
      float alpha =  2 * PI * cutoffFrequency / (sampleRate + 2 * PI * cutoffFrequency);

      float filteredAccel = 0.0;

      filteredAccel = (1-alpha) * raw_accel  + alpha * raw_accel;
      return filteredAccel;
}

float noise_removal_float (float raw_accel ){
      float sampleRate = 100.0;
      float cutoffFrequency = 20.0;
      float alpha =  2 * PI * cutoffFrequency / (sampleRate + 2 * PI * cutoffFrequency);

      float filteredAccel = 0.0;

      filteredAccel = (1-alpha) * raw_accel  + alpha * raw_accel;
      return filteredAccel;
}


void walker() {

	ICM20948_Read_Gyro(&hi2c2, &ICMData);
	ICM20948_Read_Gyro(&hi2c1, &ICMData2);
	double Gcx = ICMData.Gx;
	double Gcy = ICMData.Gy;
	double Gcz = ICMData.Gz;

	double Gcr = sqrt(pow(Gcx,2) + pow(Gcy,2) + pow(Gcz,2));

	Gcx = ICMData2.Gx;
	Gcy = ICMData2.Gy;
	Gcz = ICMData2.Gz;

	double Gcr2 = sqrt(pow(Gcx,2) + pow(Gcy,2) + pow(Gcz,2));

//    sprintf((char * )buf, "\t\t\t Gyroscope: first_gyro:%0.2f second_gyro:%0.2f  \r\n", Gcr,Gcr2);
//    send_uart((char *) buf);


	Gcr = Gcr * 10;
	Gcr2 = Gcr2 * 10;
	if ((Gcr > 905) && (Gcr2 > 905)){
		motion_count = motion_count + 1;
	} else {
		motion_count = motion_count - 1;
	}

//    sprintf((char *)buf,"motion_count : %d",motion_count);
//    send_uart((char *)buf);
	if (motion_count >= 40) {
		motion_count = 40;
	}
	if (motion_count <= -20) {
		motion_count = 0;
	}
}





/********* Sit Checker *************/
void sit_checker() {
	previousTime = currentTime;
	uint16_t alpha = 0.96;
	float av_roll ,av_pitch,av_yaw;
	float av_roll2,av_pitch2,av_yaw2;
	for(int i =0; i <= 50; i++){
		for (int i = 0; i <= 10; i++) {
			previousTime = currentTime;
			ICM20948_Read_Accel(&hi2c1,&ICMData);
			acc_x = ICMData.ACCEL_X_RAW;
			acc_y = ICMData.ACCEL_Y_RAW;
			acc_z = ICMData.ACCEL_Z_RAW;

			acc_x = noise_removal(acc_x);
			acc_y = noise_removal(acc_y);
			acc_z = noise_removal(acc_z);

			//Calculating Roll and Pitch from the accelrometer data
			acc_angle_x = (atan(acc_y/sqrt(pow(acc_x,2)+pow(acc_z,2))) * RAD_TO_DEG) - 0.58;
	      //AccErrorX ~ (0.58)
			acc_angle_y = (atan(-1 * acc_x/sqrt(pow(acc_y,2)+pow(acc_z,2))) * RAD_TO_DEG) + 1.58;
	      //AccErrorY ~ (-1.58)

			previousTime = currentTime;
			currentTime = HAL_GetTick();
			elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

			ICM20948_Read_Gyro(&hi2c1, &ICMData);
			gyro_x = gyro_x + (ICMData.Gx * elapsedTime);
			gyro_y = gyro_y + (ICMData.Gy * elapsedTime);
			gyro_z = gyro_z + (ICMData.Gz * elapsedTime);


			roll = alpha * gyro_x + (1-alpha) * acc_angle_x;
			pitch = alpha * gyro_y + (1-alpha) * acc_angle_y;
			yaw = gyro_z;


			roll = noise_removal_float(roll);
			pitch = noise_removal_float(pitch);

			av_roll += roll;
			av_pitch += pitch;

			ICM20948_Read_Accel(&hi2c2,&ICMData);
			acc_x = ICMData.ACCEL_X_RAW;
			acc_y = ICMData.ACCEL_Y_RAW;
			acc_z = ICMData.ACCEL_Z_RAW;


			acc_x = noise_removal(acc_x);
			acc_y = noise_removal(acc_y);
			acc_z = noise_removal(acc_z);


	      //Calculating Roll and Pitch from the accelrometer data
			acc_angle_x = (atan(acc_y/sqrt(pow(acc_x,2)+pow(acc_z,2))) * RAD_TO_DEG) - 0.58;
	      //AccErrorX ~ (0.58)
			acc_angle_y = (atan(-1 * acc_x/sqrt(pow(acc_y,2)+pow(acc_z,2))) * RAD_TO_DEG) + 1.58;
	      //AccErrorY ~ (-1.58)

			previousTime = currentTime;
			currentTime = HAL_GetTick();
			elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

			ICM20948_Read_Gyro(&hi2c2, &ICMData);
			gyro_x = gyro_x + (ICMData.Gx * elapsedTime);
			gyro_y = gyro_y + (ICMData.Gy * elapsedTime);
			gyro_z = gyro_z + (ICMData.Gz * elapsedTime);


	      //Complimentary filter- combine accelerometer and gyro angle values.
			roll2 = alpha * gyro_x + (1-alpha) * acc_angle_x;
			pitch2 = alpha * gyro_y + (1-alpha) * acc_angle_y;
			yaw2 = gyro_z;

			roll2 = noise_removal_float(roll2);
			pitch2 = noise_removal_float(pitch2);



			av_roll2 += roll2;
			av_pitch2 += pitch2;

		}

		walker();
			av_roll /= 10;
			av_pitch /= 10;
			av_roll2 /= 10;
			av_pitch2 /= 10;


		if(av_pitch2 <= 10){
			lying++;
		}
		if(av_pitch2 >= 55){
			tof = false;
		}
		if(av_pitch2 <= 55){
			tof = true;
			if(av_pitch >= 75 ){
				tof = false;
			}
		}



		av_pitch = 0;
		av_roll = 0 ;
		av_pitch2 = 0;
		av_roll2 = 0;

		if(tof == true){
	    	count++;
	    }
		if(motion_count == 40){
			walking++;
		}

	  }

	if (count >= 25) {
			if (lying >= 25) {
				lay = true;
				st = false;
				stnd = false;
				layTime = layTime + elapsedTime;
			} else {
				st = true;
				stnd = false;
				lay = false;
				sitTime = sitTime + elapsedTime;
			}
			count = 0;
			lying = 0;
		} else {

			layTime = layTime + elapsedTime;
			stnd = true;
			st= false;
			lay = false;
			count = 0;
			lying = 0;
			walking = 0;
		}
		if (stnd == true) {
				if (fstnd == true ){
					sprintf((char*) buf, "Standing \r\n");
					inflation_On();
				     fstnd = false;
				     fst = true;
				     flay = true;
				}
			}
		if (st == true) {
			if (fst == true){
	//			HAL_Delay(3000
				sprintf((char*) buf, "Sitting \r\n");
				inflation_Off();
				fst = false;
				fstnd = true;
				flay = true;
			}
		}
		if (lay == true) {
			if (flay == true){
				sprintf((char*) buf, "Laying \r\n");
				fst = true;
				fstnd = true;
				flay = false;
		    }
		}
		send_uart((char *) buf);
}
void stand_checker() {
	previousTime = currentTime;
		uint16_t alpha = 0.96;
		float av_roll ,av_pitch,av_yaw;
		float av_roll2,av_pitch2,av_yaw2;
		for(int i =0; i <= 50; i++){
			for (int i = 0; i <= 10; i++) {
				previousTime = currentTime;
				ICM20948_Read_Accel(&hi2c1,&ICMData);
				acc_x = ICMData.ACCEL_X_RAW;
				acc_y = ICMData.ACCEL_Y_RAW;
				acc_z = ICMData.ACCEL_Z_RAW;

				acc_x = noise_removal(acc_x);
				acc_y = noise_removal(acc_y);
				acc_z = noise_removal(acc_z);

				//Calculating Roll and Pitch from the accelrometer data
				acc_angle_x = (atan(acc_y/sqrt(pow(acc_x,2)+pow(acc_z,2))) * RAD_TO_DEG) - 0.58;
		      //AccErrorX ~ (0.58)
				acc_angle_y = (atan(-1 * acc_x/sqrt(pow(acc_y,2)+pow(acc_z,2))) * RAD_TO_DEG) + 1.58;
		      //AccErrorY ~ (-1.58)

				previousTime = currentTime;
				currentTime = HAL_GetTick();
				elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

				ICM20948_Read_Gyro(&hi2c1, &ICMData);
				gyro_x = gyro_x + (ICMData.Gx * elapsedTime);
				gyro_y = gyro_y + (ICMData.Gy * elapsedTime);
				gyro_z = gyro_z + (ICMData.Gz * elapsedTime);


				roll = alpha * gyro_x + (1-alpha) * acc_angle_x;
				pitch = alpha * gyro_y + (1-alpha) * acc_angle_y;
				yaw = gyro_z;


				roll = noise_removal_float(roll);
				pitch = noise_removal_float(pitch);

				av_roll += roll;
				av_pitch += pitch;

				ICM20948_Read_Accel(&hi2c2,&ICMData);
				acc_x = ICMData.ACCEL_X_RAW;
				acc_y = ICMData.ACCEL_Y_RAW;
				acc_z = ICMData.ACCEL_Z_RAW;


				acc_x = noise_removal(acc_x);
				acc_y = noise_removal(acc_y);
				acc_z = noise_removal(acc_z);


		      //Calculating Roll and Pitch from the accelrometer data
				acc_angle_x = (atan(acc_y/sqrt(pow(acc_x,2)+pow(acc_z,2))) * RAD_TO_DEG) - 0.58;
		      //AccErrorX ~ (0.58)
				acc_angle_y = (atan(-1 * acc_x/sqrt(pow(acc_y,2)+pow(acc_z,2))) * RAD_TO_DEG) + 1.58;
		      //AccErrorY ~ (-1.58)

				previousTime = currentTime;
				currentTime = HAL_GetTick();
				elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

				ICM20948_Read_Gyro(&hi2c2, &ICMData);
				gyro_x = gyro_x + (ICMData.Gx * elapsedTime);
				gyro_y = gyro_y + (ICMData.Gy * elapsedTime);
				gyro_z = gyro_z + (ICMData.Gz * elapsedTime);


		      //Complimentary filter- combine accelerometer and gyro angle values.
				roll2 = alpha * gyro_x + (1-alpha) * acc_angle_x;
				pitch2 = alpha * gyro_y + (1-alpha) * acc_angle_y;
				yaw2 = gyro_z;

				roll2 = noise_removal_float(roll2);
				pitch2 = noise_removal_float(pitch2);



				av_roll2 += roll2;
				av_pitch2 += pitch2;

			}

			walker();
				av_roll /= 10;
				av_pitch /= 10;
				av_roll2 /= 10;
				av_pitch2 /= 10;


			if(av_pitch2 <= 10){
				lying++;
			}
			if(av_pitch2 >= 55){
				tof = false;
			}
			if(av_pitch2 <= 55){
				tof = true;
				if(av_pitch >= 75 ){
					tof = false;
				}
			}



			av_pitch = 0;
			av_roll = 0 ;
			av_pitch2 = 0;
			av_roll2 = 0;

			if(tof == true){
		    	count++;
		    }
			if(motion_count == 40){
				walking++;
			}

		  }

		if (count >= 25) {
					if (lying >= 25) {
						lay = true;
						st = false;
						stnd = false;
						layTime = layTime + elapsedTime;
					} else {
						st = true;
						stnd = false;
						lay = false;
						sitTime = sitTime + elapsedTime;
					}
					count = 0;
					lying = 0;
				} else {

					layTime = layTime + elapsedTime;
					stnd = true;
					st= false;
					lay = false;
					count = 0;
					lying = 0;
					walking = 0;
				}
				if (stnd == true) {
						if (fstnd == true ){
							sprintf((char*) buf, "Standing \r\n");
							inflation_On();
						     fstnd = false;
						     fst = true;
						     flay = true;
						}
					}
				if (st == true) {
					if (fst == true){
			//			HAL_Delay(3000
						sprintf((char*) buf, "Sitting \r\n");
						inflation_Off();
						fst = false;
						fstnd = true;
						flay = true;
					}
				}
				if (lay == true) {
					if (flay == true){
						sprintf((char*) buf, "Laying \r\n");
						fst = true;
						fstnd = true;
						flay = false;
				    }
				}

				send_uart((char *) buf);
}
int main(void)
{

// Initialization of the Peripherals
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();




  //Initialization of the Timers ( For PWM Generation)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  //For BLDC Initialization
//  for(int i =0; i <= 100; i++ ){
//          TIM2->CCR2 = i;
//          TIM1->CCR1 = i;
//          HAL_Delay(10);
//   }




  HAL_Delay(4000);
  TIM1->CCR1 = 200;
  TIM2->CCR2 = 200;
  HAL_Delay(4000);
  TIM1->CCR1 = 100;
  TIM2->CCR2 = 100;
  HAL_Delay(3000);
  TIM1->CCR1 =  0;
  TIM2->CCR2  = 0;
//  HAL_Delay(100);

  HAL_Delay(1000);
  TIM1->CCR4 = 140;
  TIM2->CCR1 = 140;

  	sprintf((char*) buf, "One \r\n\n");

  //Initializing the ICM20948 for the stomach
  	ICM20948_Init(&hi2c1);
  	ICM20948_Init(&hi2c1);
  	sprintf((char*) buf, "Two \r\n\n");
  	HAL_Delay(200);
  //Initializing the ICM20948 for the thigh
//   inflation_On();
  	ICM20948_Init(&hi2c2);
  	ICM20948_Init(&hi2c2);
  	sprintf((char*) buf, "Three Go.. \r\n\n");
  	HAL_Delay(200);

//  inflation_On();
  // Reading  the acceleration
  	ICM20948_Read_Accel(&hi2c1, &ICMData);
   //Scaling the Value of accel_x_raw using map function for scaling.
//  	int xAng = map(ICMData.ACCEL_X_RAW, minVal, maxVal, -90, 90);
//  	int yAng = map(ICMData.ACCEL_Y_RAW, minVal, maxVal, -90, 90);
//  	int zAng = map(ICMData.ACCEL_Z_RAW, minVal, maxVal, -90, 90);

  	//Taking the initial_angle_x for the initial_angle_x to check the function
//  	initial_angle_x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
//  	initial_angle_y = RAD_TO_DEG * (atan2(-xAng, -yAng) + PI);
//  	initial_angle_z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);


  	//Dc Noise Removal
  	for( int i =0 ; i <= 50; i++){
  		ICM20948_Read_Accel(&hi2c1, &ICMData);
  		dc_x1 += ICMData.ACCEL_X_RAW;
  		dc_y1 += ICMData.ACCEL_Y_RAW;
  		dc_z1 += ICMData.ACCEL_Z_RAW;

  		ICM20948_Read_Accel(&hi2c2, &ICMData);
  		dc_x2 += ICMData.ACCEL_X_RAW;
  		dc_y2 += ICMData.ACCEL_Y_RAW;
  		dc_z2 += ICMData.ACCEL_Z_RAW;
  	}

    dc_x1 /= 50;
    dc_y1 /= 50;
    dc_z1 /= 50;
    dc_x2 /= 50;
    dc_y2 /= 50;
    dc_z2 /= 50;
  	/* USER CODE END 2 */
  	ICM20948_Read_Accel(&hi2c2, &ICMData2);
//  	inflation_On();
//    inflation_On();
  	while (1) {

  		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET){
  			if(flag == 1){
  				inflation_On();
  			  	flag = 0 ;
  			  	}
  			else
  			{
  				inflation_Off();
  			  	flag = 1 ;
  			 }
  		}
  		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET){
  		  			held_press();
  		 }
  		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET){
  			 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
  			 HAL_Delay(500);
  		}


  		if (trigger == true) {
  			//This function initiate when person is in sitting condition
  			sit_checker();
  		} else {
  			//This function init
  			stand_checker();
  		}
    }
}
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//
//    if(GPIO_Pin == GPIO_PIN_1){
//
//    	HAL_Delay(200);
//    	    	 uint32_t currentTime = HAL_GetTick();
//    	    	 if ((currentTime - lastDebounceTime) >= DEBOUNCE_DELAY_MS) {
//    	    		 lastDebounceTime = currentTime;
//    	    		 held_press();
//    	    	 }
//    }
//    if(GPIO_Pin == GPIO_PIN_2){
//    	HAL_Delay(200);
//    	    	 uint32_t currentTime = HAL_GetTick();
//    	    	 if ((currentTime - lastDebounceTime) >= DEBOUNCE_DELAY_MS) {
//    	    		 lastDebounceTime = currentTime;
//    	    		 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
//    	    	 }
//    }
//}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
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
