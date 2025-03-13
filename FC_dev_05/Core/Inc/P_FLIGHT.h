/*
 * P_FLIGHT.h
 *
 *  Created on: Nov 6, 2024
 *      Author: Sezakiaoi
 */

#ifndef INC_P_FLIGHT_H_
#define INC_P_FLIGHT_H_

#include "FLIGHT.h"

class P_FLIGHT : private FLIGHT{

	public:

		uint8_t Init();
		uint8_t Wait_Arm();
		void Dis_Arm();
		uint8_t Arming();

		void sbus_input(uint16_t* input_data);


	private:

};

#endif /* INC_P_FLIGHT_H_ */
