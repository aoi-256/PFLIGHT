/*
 * P_FLIGHT.cpp
 *
 *  Created on: Nov 6, 2024
 *      Author: Sezakiaoi
 */

#include <P_FLIGHT.h>
#include <gpio.h>

//電源入力時の処理
uint8_t P_FLIGHT::Init(){

	//SBUSの受信チェック

	//PWMの起動
	FLIGHT::PWM_Init();

	//PIDの起動
	FLIGHT::PID_Calc();

	//センサーへのWHO_AM_I
	FLIGHT::ICM42688P_Setup();

	return 0;
}

//arm待機用の関数
uint8_t P_FLIGHT::Wait_Arm(){

	//armするまでここでループ
	while(FLIGHT::IsArm()){

		HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_1);

		HAL_Delay(500);
	}

	FLIGHT::PWM_Init();

	return 0;
}

//arm中の処理
uint8_t P_FLIGHT::Arming(){

		//arm中はここでループ
		while(FLIGHT::IsArm()){

			//フェイルセーフの判定
			if(FLIGHT::CheckFailSafe()){

				//arm状態の解除とSBUSのバッファークリア
				FLIGHT::SBUS_Reset();
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0, GPIO_PIN_SET);
				return 1;
			}

			//armLEDをつける
			HAL_GPIO_WritePin(GPIOH,GPIO_PIN_1, GPIO_PIN_SET);

			//フェイルセーフLED（デバック）
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0, GPIO_PIN_RESET);

			//センサーの値取得
			FLIGHT::ICM42688P_GetData();

			//カルマンフィルター

			//PIDでPWMの計算
			FLIGHT::PID_Calc();

			//PWMの出力
			FLIGHT::PWM_Generate();
		}
	return 0;
}

//arm終了時の処理
void P_FLIGHT::Dis_Arm(){

	//モーターの停止
	FLIGHT::PWM_Stop();

	//armLEDをけす
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_1, GPIO_PIN_RESET);

}

//FLIGHT::sbus_input() へのデータの受け渡し
void P_FLIGHT::sbus_input(uint16_t* SbusData){

	FLIGHT::SBUS_Set(SbusData);
}

