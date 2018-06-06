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

void Drive::PIDMove(double Dtot, double Vf_at_end, double CoW, double a, int timeout, double sensitivity){
	Dtot = Dtot *(4096/CoW);
	double s;
	double setpoint = encoder.Get();
	//Leaner motion equation
	if(Vf_at_end	 == 0.0){
		s = (2.0 * Dtot) + ((-a*std::pow(2*Dtot, 2.0))/std::pow(Uvalues[0], 2.0));
		while(s > encoder.Get()){
			setpoint += sensitivity;
			Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
			Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
		}
		while(s <= encoder.Get()){
			Left_Front.StopMotor();
			Right_Front.StopMotor();
		}

	} else{
		s = (Uvalues[0] * (2.0 * Dtot/(Uvalues[0] + Vf_at_end))) + ((-a*std::pow(2*Dtot, 2.0))/std::pow(Uvalues[0], 2.0));
		Drive::setU(Vf_at_end);

	}

}

void Drive::setU(double setU){
	Uvalues[0] = setU;
}












