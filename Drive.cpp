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

void Drive::ArcadeDrive(int deadzone, double sensitivity){
	if(xbox1.frc::Joystick::GetX(frc::GenericHID::JoystickHand::kRightHand)> deadzone){
		//Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Set(ctre::phoenix::motorcontrol::ControlMode::Position, )
	}
	else{

	}
	if(xbox1.frc::Joystick::GetY(frc::GenericHID::JoystickHand::kLeftHand)> deadzone){

	}
	else{

	}
}

