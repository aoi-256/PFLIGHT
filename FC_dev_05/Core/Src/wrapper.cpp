#include "wrapper.hpp"
#include "usart.h"
#include "tim.h"
#include "SBUS.h"
#include "P_FLIGHT.h"

P_FLIGHT pflight;
SBUS sbus;

//sbusの受信用の変数
uint8_t sbus_buffer[25] = {};
uint16_t sbus_data[11] = {};

void init(){

	//電源入力時の初期処理
	pflight.Init();

	//SBUS割り込みの設定
	HAL_UART_Receive_DMA(&huart2, sbus_buffer, 25);

	//arm待機（arm待機中はここでループ）
	pflight.Wait_Arm();
}

void loop(){

	//飛行中の処理の実行（arm中はここでループ）
	pflight.Arming();

	//armが中断された時の処理(処理の終了を確認するまでループ）
	pflight.Dis_Arm();

	//armの待機（arm待機中はここでループ）
	pflight.Wait_Arm();

	//armすると上に戻って再度飛行
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	//uart SBUSの受信
	if(huart==&huart2){

		//受信成功した場合のみ値を書きこむ
		if(sbus.encode(sbus_buffer, sbus_data)){

			//値の書き込み
			pflight.sbus_input(sbus_data);
		}

		//受信の再開
		HAL_UART_Receive_DMA(&huart2, sbus_buffer, 25);
	}
}



