/*
 * FLIGHT.h
 *
 *  Created on: Mar 13, 2025
 *      Author: Sezakiaoi
 */

#ifndef SRC_FLIGHT_H_
#define SRC_FLIGHT_H_

#include <cstdint>

class FLIGHT {

	public:

		struct Channel{

			uint8_t Pitch = 0;
			uint8_t Roll  = 1;
			uint8_t Yaw	  = 2;
			uint8_t Arm = 5;
		};

		//SBUS
		void SBUS_Set(uint16_t* SbusData);
		void SBUS_Reset();
		void SBUS_Convert();
		uint8_t CheckFailSafe();

		//ARM
		uint8_t IsArm();

		//ICM42688P
		uint8_t ICM42688P_Setup();
		void ICM42688P_GetData();

		//BM1422AGMV
		uint8_t BM1422AGMV_Setup();
		void ICM42688P_GetData(int16_t MagBuffer[3]);

		//Karman
		void Karman_Setup();
		void Karman_Calc();

		//PID
		void PID_Setup();
		void PID_Calc();
		void PID_Reset();

		//PWM
		void PWM_Init();
		void PWM_Generate();
		void PWM_Stop();

	private:

		uint16_t SbusData[10] = {};
		float AccelData[3] = {};
		float GyroData[3] = {};
		float AngleData[3] = {};
		float TargetAngle[3] = {};
		uint8_t FaliSafeCount = 0;

};

#endif /* SRC_FLIGHT_H_ */
