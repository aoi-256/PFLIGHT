/*
 * FLIGHT.cpp
 *
 *  Created on: Nov 5, 2024
 *      Author: Sezakiaoi
 */

#include <FLIGHT.h>
#include <tim.h>
#include "usart.h"
#include "PID.h"
#include "ICM42688P.h"

//----------------------instance----------------------//

PID pitch;
PID roll;
PID yaw;
ICM42688P icm(&hspi1, GPIOA, GPIO_PIN_4);

//----------------------arm----------------------//


/**
 * @brief Armの判定
 *
 * @return Arm状態 0: DisArm, 1: Arm
 *
 * Armスイッチの値を監視し、状態を返します
 */
uint8_t FLIGHT::IsArm(){

	if(SbusData[5] > 1000){

		return 1;
	}
	return 0;
}

//----------------------SBUS----------------------//

/**
 * @brief SBUSデータの取り込み
 *
 * @param[in]　エンコード済みのSBUSデータ
 */
void FLIGHT::SBUS_Set(uint16_t* SbusData){

	for(uint8_t i=0; i<10; i++){

		if(SbusData[i] != 0){

			this->SbusData[i] = SbusData[i];
		}
	}
	FaliSafeCount = 0;
}

/**
 * @brief SBUSデータのリセット
 *
 * フェイルセーフ時に使用
 */
void FLIGHT::SBUS_Reset(){

	for(uint8_t i=0; i<10; i++){

		SbusData[i] = 0;
	}
}

/**
 * @brief SBUSデータから目標角度を計算
 *
 * 値から1500を引いたあと、SBUSの最大値と最小値が角度の最大値と最小値（±60°）になるようにする
 */
void FLIGHT::SBUS_Convert(){

	TargetAngle[0] = (SbusData[0] - 1500) * (60.0 / 500.0);
	TargetAngle[0] = (SbusData[1] - 1500) * (60.0 / 500.0);
	TargetAngle[0] = (SbusData[2] - 1500) * (60.0 / 500.0);
}

/**
 * @brief フェイルセーフの判定
 *
 * @return 0: 正常  1:フェイルセーフ発動
 *
 * Arm時、毎ループごとに、SBUSのデータが更新されると０になるカウントをインクリメント
 * 値が一定以上になった場合フェイルセーフを発動する
 */
uint8_t FLIGHT::CheckFailSafe(){

	FaliSafeCount ++;

	if(FaliSafeCount >= 1000){

		FaliSafeCount = 0;
		return 1;
	}
	return 0;
}
//----------------------ICM42688P----------------------//

/**
 * @brief ICM42688Pの初期設定
 *
 * @return 0: 正常  1:通信失敗
 *
 * ICM42688PのWIAレジスタを用いて通信を確認し、
 * 問題がない場合は、センサーの初期設定を行う
 */
uint8_t FLIGHT::ICM42688P_Setup(){

	if(icm.Connection() == 1){

		//失敗時の出力
		return 1;
	}

	icm.AccelConfig(icm.ACCEL_Mode::LowNoize, icm.ACCEL_SCALE::SCALE02g, icm.ACCEL_ODR::ODR01000hz, icm.ACCEL_DLPF::ODR40);
	icm.GyroConfig(icm.GYRO_MODE::LowNoize, icm.GYRO_SCALE::Dps0250, icm.GYRO_ODR::ODR01000hz, icm.GYRO_DLPF::ODR40);

	return 0;
}

/**
 * @brief ICM42688Pから6軸データを取得
 *
 * @param AccelData 加速度データ
 * @param AccelData ジャイロデータ
 *
 * 単位はm/s^2とdps
 */
void FLIGHT::ICM42688P_GetData(){

	icm.GetData(AccelData, GyroData);
}

//----------------------Karman----------------------//

void Karman_Calc(){


}

//----------------------PWM関係----------------------//

//PWMの開始と初期値の入力
void FLIGHT::PWM_Init(){

	//モーター
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	//サーボ
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

	//モーター
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_4, 0);

	__HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_4, 0);

	//サーボ
	__HAL_TIM_SET_COMPARE(&htim12 , TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim12 , TIM_CHANNEL_2, 1000);
}

//PWMの値の更新
void FLIGHT::PWM_Generate(){

	//モーター
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_4, 0);

	__HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_4, 0);

	//サーボ
	__HAL_TIM_SET_COMPARE(&htim12 , TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim12 , TIM_CHANNEL_2, 1000);
}

void FLIGHT::PWM_Stop(){

	//モータの停止
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

	//サーボのリセット
	__HAL_TIM_SET_COMPARE(&htim12 , TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim12 , TIM_CHANNEL_2, 1000);
}

//----------------------PID----------------------//

void FLIGHT::PID_Setup(){

	//mane P I D Goal
	pitch.Setup(0.01, 0, 0, 0);
	roll.Setup(0.01, 0, 0, 0);
	yaw.Setup(0.01, 0, 0, 0);
}

void FLIGHT::PID_Calc(){

	pitch.Calc(AngleData[0], TargetAngle[0]);
	roll.Calc(AngleData[1], TargetAngle[1]);
	yaw.Calc(AngleData[2], TargetAngle[2]);
}

void FLIGHT::PID_Reset(){

	pitch.Reset();
	roll.Reset();
	yaw.Reset();
}

