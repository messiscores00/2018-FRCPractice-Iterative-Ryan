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
	a = a * 12 * (4096/CoW);
	sensitivity = sensitivity * 12 * (4096/CoW);
	double setpoint = encoder.Get();
	if(Vf_at_end == 0.0){
		//Leaner motion equation
		s = (2.0 * Dtot) + ((-a*std::pow(2*Dtot, 2.0))/std::pow(Uvalue, 2.0));
		while(s > encoder.Get() && counter.Get()*1000 > timeout){
			if(Drive::ASecond()==true){
				setpoint += sensitivity;
			}
			Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
			Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
		}
		while(s <= encoder.Get() && counter.Get()*1000 > timeout){
			Left_Front.StopMotor();
			Right_Front.StopMotor();
		}

	} else{
		//Leaner motion equation
		s = (Uvalue * (2.0 * Dtot/(Uvalue + Vf_at_end))) + ((-a*std::pow(2*Dtot, 2.0))/std::pow(Uvalue, 2.0));
		Uvalue = Vf_at_end;
		while(s > encoder.Get() && counter.Get()*1000 > timeout){
			if(Drive::ASecond()==true){
				setpoint += sensitivity;
			}
			Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
			Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
		}
		while(s <= encoder.Get() && counter.Get()*1000 > timeout){
			Left_Front.StopMotor();
			Right_Front.StopMotor();
		}
	}

}

bool Drive::ASecond(){
	time(&now);
	end = now + 1;
	//if seconds = 1 sec the time that has passed is 0.
	//if seconds = 0 sec the time that has passed is 1 sec.
	seconds = difftime(now,end);
	if(seconds > 0){
		return false;
	}else{
		return true;
	}
}











