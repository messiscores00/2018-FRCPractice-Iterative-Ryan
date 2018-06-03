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

void Drive::ArcadeDrive(double deadzone, double sensitivity){

	xbox1.SetXChannel(4);
	xbox1.SetYChannel(1);
	double left_right = -1 * sensitivity * xbox1.GetX(frc::GenericHID::JoystickHand::kRightHand);
	double forw_back  = sensitivity * xbox1.GetY(frc::GenericHID::JoystickHand::kLeftHand);

	if (fabs(left_right) < deadzone)
		left_right = 0;
	if (fabs(forw_back) < deadzone)
		forw_back = 0;

	_diffDrive.ArcadeDrive(forw_back, left_right, false);

}

void Drive::PIDForw(double u, double Dtot, double Vf, double CoW){
	Dtot = Dtot/CoW * 4096;
	//Leaner motion equation
	//s = displacement
	double s = (u * ((2*Dtot)/(u+Vf)) + (((encoder.frc::Encoder::Get()/(counter.Get()^2))*(4*Dtot))/2));

	if(s > Dtot){
	}
}











