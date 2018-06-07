/*
 * Drive.cpp
 *
 *  Created on: May 30, 2018
 *      Author: Ryan Sheehy
 */

#include "Drive.h"

Drive::Drive() {
}

Drive::~Drive() {
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
	//put the Y-axis on the left joystick
	//put the X-axis on the right joystick
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
	//convert variables into ticks
	Dtot = Dtot *(4096/CoW);
	a = a * 12 * (4096/CoW);
	Vf_at_end = Vf_at_end * 12 * (4096/CoW);
	sensitivity = sensitivity * 12 * (4096/CoW);
	//sets the setpoint to the value of the encoder at that time
	setpoint = encoder.Get();

	if(Vf_at_end == 0.0){
		//linear motion equation
		s = (2.0 * Dtot) + ((-a*std::pow(2*Dtot, 2.0))/std::pow(Uvalue, 2.0));
	}else{
		//same equation, but with the final velocity can be set
		s = (Uvalue * (2.0 * Dtot/(Uvalue + Vf_at_end))) + ((-a*std::pow(2*Dtot, 2.0))/std::pow(Uvalue, 2.0));
		//sets the final velocity so it can be used for the next call of this function
		Uvalue = Vf_at_end;
	}

	// * by 1000 because it outputs in seconds while timeout is in miliseconds
		while(s > encoder.Get() && counter.Get()*1000 > timeout){
			if(Drive::ASecond()==true){
				//every second add the sensitivity to the setpoint
				setpoint += sensitivity;
			}
			//moves robot to the setpoint
			Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
			Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
		}
		if(s <= encoder.Get() && counter.Get()*1000 > timeout){
			//once you have reached the point where you need to stop (displacement) you stop
			Left_Front.StopMotor();
			Right_Front.StopMotor();
		}
}

void Drive::PIDTurn(double Vf_at_end, double CoW, double a, int timeout, double sensitivity, double radius_left, double radius_right, int angle){
	//converting to ticks
	sensitivity = sensitivity * 12 * (4096/CoW);
	Vf_at_end = Vf_at_end * 12 * (4096/CoW);
	radius_left = radius_left * 12 * (4096/CoW);
	radius_right = radius_right * 12 * (4096/CoW);
	//finding the total distance for each side of the robot
		//the reason why it is different is because you are going around a circle. This causes the ark length of the right side and the left side to be different because the radius is different.
	ArkLeng_left = 2 * 3.14 * radius_left * (angle/360);
	ArkLeng_right = 2 * 3.14 * radius_right * (angle/360);
	//setting setpoint to where the robot is
	setpoint = encoder.Get();

	//linear motion equation for each side of the robot.
	if(Vf_at_end == 0.0){
		Sleft = (2.0 * ArkLeng_left) + ((-a*std::pow(2*ArkLeng_left, 2.0))/std::pow(Uvalue, 2.0));
		Sright = (2.0 * ArkLeng_right) + ((-a*std::pow(2*ArkLeng_right, 2.0))/std::pow(Uvalue, 2.0));
	}else{
		Sleft = (Uvalue * (2.0 * ArkLeng_left/(Uvalue + Vf_at_end))) + ((-a*std::pow(2*ArkLeng_left, 2.0))/std::pow(Uvalue, 2.0));
		Sright = (Uvalue * (2.0 * ArkLeng_right/(Uvalue + Vf_at_end))) + ((-a*std::pow(2*ArkLeng_right, 2.0))/std::pow(Uvalue, 2.0));
		Uvalue = Vf_at_end;
	}

	//checks which ark length is greater
		if(ArkLeng_left > ArkLeng_right){
			while(Sleft > encoder.Get() && counter.Get()*1000 > timeout){
				if(Drive::ASecond()==true){
					setpoint += sensitivity;
				}
				Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
				//put the other side of the robot separate in an "if" statement because one side will stop sooner, but you want one side to continue while the other side keeps driving.
				if(Sright > encoder.Get()){
					Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
				}else {
					Right_Front.StopMotor();
				}
			}
			if(Sleft <= encoder.Get() && counter.Get()*1000 > timeout){
				Left_Front.StopMotor();
				Right_Front.StopMotor();
			}
		} else if(ArkLeng_left < ArkLeng_right) {
			while(Sright > encoder.Get() && counter.Get()*1000 > timeout){
				if(Drive::ASecond()==true){
					setpoint += sensitivity;
				}
				Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
				//put the other side of the robot separate in an "if" statement because one side will stop sooner, but you want one side to continue while the other side keeps driving.
				if(Sleft > encoder.Get()){
					Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
				}else {
					Left_Front.StopMotor();
				}
			}
			if(Sright <= encoder.Get() && counter.Get()*1000 > timeout){
				Right_Front.StopMotor();
				Left_Front.StopMotor();
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











