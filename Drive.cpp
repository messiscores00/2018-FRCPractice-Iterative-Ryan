/*
 * Drive.cpp
 *
 *  Created on: May 30, 2018
 *      Author: Ryan Sheehy
 */

#include "Drive.h"

Drive::Drive() {
	// TODO Auto-generated constructor stub

}

Drive::~Drive() {
	// TODO Auto-generated destructor stub
}
void Drive::PIDenable(int P, int I, int D, int F){
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kP(0, P, 0);
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kI(0, I, 0);
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kD(0, D, 0);
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kF(0, F, 0);
	Right_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kP(0, P, 0);
	Right_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kI(0, I, 0);
	Right_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kD(0, D, 0);
	Right_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kF(0, F, 0);
}

void Drive::PIDdisable(){
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kP(0, 0, 0);
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kI(0, 0, 0);
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kD(0, 0, 0);
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kF(0, 0, 0);
	Right_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kP(0, 0, 0);
	Right_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kI(0, 0, 0);
	Right_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kD(0, 0, 0);
	Right_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kF(0, 0, 0);
}

void Drive::ArcadeDrive(int deadzone, double sensitivity){

}

