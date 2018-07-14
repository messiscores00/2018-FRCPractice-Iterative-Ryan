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
//----------------------------------------------------------------------------------------------
void Drive::PIDenable(int P, int I, int D, int F){
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kP(0, P, 0);
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kI(0, I, 0);
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kD(0, D, 0);
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kF(0, F, 0);
	Right_Back.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kP(0, P, 0);
	Right_Back.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kI(0, I, 0);
	Right_Back.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kD(0, D, 0);
	Right_Back.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kF(0, F, 0);
}
//----------------------------------------------------------------------------------------------
void Drive::PIDdisable(){
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kP(0, 0, 0);
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kI(0, 0, 0);
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kD(0, 0, 0);
	Left_Front.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kF(0, 0, 0);
	Right_Back.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kP(0, 0, 0);
	Right_Back.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kI(0, 0, 0);
	Right_Back.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kD(0, 0, 0);
	Right_Back.ctre::phoenix::motorcontrol::can::WPI_TalonSRX::Config_kF(0, 0, 0);
}
//----------------------------------------------------------------------------------------------
void Drive::ArcadeDrive(double deadzone, double xSpeed, double zRotation, bool squaredInputs, double Leftsensitivity, double Rightsensitivity){
	//deadzone is from 1 to 0 where 1 = 100%
	//xSpeed is the Y axis on the controller. note: it is called xSpeed because the robot wheels rotates on the x axis
	//zRotation is the x axis on the controller. note: it is called zRotation because the robot pivots on the z axis
	//squaredInputs is exponential growth
	//sensitivity is the % that you want to go at. From 1 to 0 where 1 = 100%
	double leftMotorOutput;
	double rightMotorOutput;

	if (fabs(xSpeed) <= deadzone){
	    xSpeed = 0;
	}
	if(fabs(zRotation) <= deadzone){
		zRotation = 0;
	}

	if (squaredInputs) {
	   xSpeed = std::copysign(xSpeed * xSpeed, xSpeed);
	   zRotation = std::copysign(zRotation * zRotation, zRotation);
	}

	double maxInput = std::copysign(std::max(std::abs(xSpeed), std::abs(zRotation)), xSpeed);

	if (xSpeed >= 0.0){
	// First quadrant, else second quadrant
		if (zRotation >= 0.0){
			leftMotorOutput = maxInput;
			rightMotorOutput = xSpeed - zRotation;
		}
		else {
			leftMotorOutput = xSpeed + zRotation;
			rightMotorOutput = maxInput;
		}
	}
	else {
	    // Third quadrant, else fourth quadrant
	    if (zRotation >= 0.0){
	       leftMotorOutput = xSpeed + zRotation;
	       rightMotorOutput = maxInput;
	    }
	    else {
	       leftMotorOutput = maxInput;
	       rightMotorOutput = xSpeed - zRotation;
	    }
	}
	Left_Front.Set(leftMotorOutput * Leftsensitivity);
	Right_Back.Set(-rightMotorOutput * Rightsensitivity);
}
//----------------------------------------------------------------------------------------------
void Drive::PIDMove(double Dtot, double Vf_at_end, double CoW, double acceleration, int timeout, double MaxVelocity){
	//Dtot = total distance in inches
	//Vf_at_end = final velocity in ft/sec
	//CoW = circumference of Wheel in inches
	//acceleration = max deceleration in ft/s^2 note: do not make this negative
	//timeout in milliseconds
	//MaxVelocity in ft/s

	double ticksPERin = 4096/CoW;

	//convert variables into ticks
	Dtot = Dtot * ticksPERin;
	acceleration = acceleration * 12 * ticksPERin;
	Vf_at_end = Vf_at_end * 12 * ticksPERin;
	MaxVelocity = MaxVelocity * 12 * ticksPERin;

	//zero the encoders & time
	PIDdisable();
	Left_Front.SetSelectedSensorPosition(0, 0, 0);
	Right_Back.SetSelectedSensorPosition(0, 0, 0);
	Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0);
	Right_Back.Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0);
	PIDenable(P,I,D,F);
	counter.Reset();

	//variables
	double setpoint = 0;
	double lasttime = counter.GetFPGATimestamp();
	double curtime;
	double difference;
	double s; //displacement
	double VelocityPresent = 0.0;


	while(setpoint < Dtot && counter.GetFPGATimestamp()*1000 < timeout){

		s = ((Vf_at_end * Vf_at_end) - (VelocityPresent * VelocityPresent))/(2 * -acceleration);

		//timing
		curtime = counter.GetFPGATimestamp();
		difference =  curtime - lasttime;
		lasttime = curtime;

		if(s < Dtot - setpoint){
			//accelerate {
			if(MaxVelocity <= VelocityPresent){
				VelocityPresent = MaxVelocity;
			}else{
				VelocityPresent += acceleration * difference;
			}

			setpoint += VelocityPresent * difference;
			if(setpoint > Dtot){
				setpoint = Dtot;
			}

			//frc::DriverStation::ReportWarning("encoderLeft: " + std::to_string(encoderLeft()));
			//frc::DriverStation::ReportWarning("encoderRight: " + std::to_string(encoderRight()));
			//frc::DriverStation::ReportWarning("setpoint: " + std::to_string(std::copysign(setpoint, Dtot)));

			Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setpoint, Dtot) * -1);
			Right_Back.Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setpoint, Dtot));
			//}
		} else {
			//decelerate {
			VelocityPresent -= acceleration* difference;
			setpoint += VelocityPresent * difference;
			if(setpoint > Dtot){
				setpoint = Dtot;
			}

			//frc::DriverStation::ReportWarning("encoderLeft: " + std::to_string(encoderLeft()));
			//frc::DriverStation::ReportWarning("encoderRight: " + std::to_string(encoderRight()));
			//frc::DriverStation::ReportWarning("setpoint: " + std::to_string(std::copysign(setpoint, Dtot)));

			Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setpoint, Dtot) * -1);
			Right_Back.Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setpoint, Dtot));
			//}
		}
	}
}
//----------------------------------------------------------------------------------------------
void Drive::PIDTurn(double Vf_at_end, double CoW, double accelerationLeft, double  accelerationRight, int timeout,double MaxVelocity, double a_left, double b_left, double a_right, double b_right, double angle){
	//a_left, b_left, a_right, b_right in inches
	//a/b is from the equation of an elipse
	//angle in degrees
	//Vf_at_end = final velocity in ft/sec
	//CoW = circumference of Wheel in inches
	//acceleration = max deceleration in ft/s^2 note: do not make this negative
	//timeout in milliseconds
	//MaxVelocity in ft/s

	double ticksPERin = 4096/CoW;

	//converting variables to ticks
	Vf_at_end = Vf_at_end * 12 * ticksPERin;
	a_left = a_left *  ticksPERin;
	b_left = b_left *  ticksPERin;
	a_right = a_right  * ticksPERin;
	b_right = b_right  * ticksPERin;
	accelerationLeft = accelerationLeft * 12 * ticksPERin;
	accelerationRight = accelerationRight * 12 * ticksPERin;
	Vf_at_end = Vf_at_end * 12 * ticksPERin;
	MaxVelocity = MaxVelocity * 12 * ticksPERin;

	//variables
	double setpointLeft = 0;
	double setpointRight = 0;
	double lasttime = counter.GetFPGATimestamp();
	double curtime;
	double difference;
	double Sleft;
	double Sright;
	double ArkLeng_left = (2.0 * 3.14 * sqrt(((a_left * a_left) + (b_left * b_left))/2)) * (angle/360);
	double ArkLeng_right = (2.0 * 3.14 * sqrt(((a_right * a_right) + (b_right * b_right))/2))  * (angle/360);
	double VelocityPresentLeft = 0.0;
	double VelocityPresentRight = 0.0;

	//zero the encoders & time
	PIDdisable();
	Left_Front.SetSelectedSensorPosition(0, 0, 0);
	Right_Back.SetSelectedSensorPosition(0, 0, 0);
	Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0);
	Right_Back.Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0);
	PIDenable(P,I,D,F);
	counter.Reset();

		while(setpointLeft < ArkLeng_left && /*setpointRight < ArkLeng_right &&*/ counter.GetFPGATimestamp()*1000 < timeout){

			Sleft = ((Vf_at_end * Vf_at_end) - (VelocityPresentLeft * VelocityPresentLeft))/(2 * -accelerationLeft);
			Sright = ((Vf_at_end * Vf_at_end) - (VelocityPresentRight * VelocityPresentRight))/(2 * -accelerationRight);

			//timing
			curtime = counter.GetFPGATimestamp();
			difference = curtime - lasttime;
			lasttime = curtime;

			if(Sleft < ArkLeng_left - setpointLeft){
				//accelerate Left{
				if(MaxVelocity <= VelocityPresentLeft){
					VelocityPresentLeft = MaxVelocity;
				}else{
					VelocityPresentLeft += accelerationLeft * difference;
				}

				setpointLeft += VelocityPresentLeft * difference;
				if(setpointLeft > ArkLeng_left){
					setpointLeft = ArkLeng_left;
				}

				frc::DriverStation::ReportWarning("encoderLeft: " + std::to_string(encoderLeft()));
				frc::DriverStation::ReportWarning("encoderRight: " + std::to_string(encoderRight()));
				//frc::DriverStation::ReportWarning("setpointLeft: " + std::to_string(std::copysign(setpointLeft, ArkLeng_left)));
				//frc::DriverStation::ReportWarning("setpointRight: " + std::to_string(std::copysign(setpointRight, ArkLeng_right)));

				Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setpointLeft, ArkLeng_left) * -1);
				//}

			} else {
				//decelerate {
				VelocityPresentLeft -= accelerationLeft * difference;
				setpointLeft += VelocityPresentLeft * difference;
				if(setpointLeft > ArkLeng_left){
					setpointLeft = ArkLeng_left;
				}

				frc::DriverStation::ReportWarning("encoderLeft: " + std::to_string(encoderLeft()));
				frc::DriverStation::ReportWarning("encoderRight: " + std::to_string(encoderRight()));
				//frc::DriverStation::ReportWarning("setpointLeft: " + std::to_string(std::copysign(setpointLeft, ArkLeng_left)));
				//frc::DriverStation::ReportWarning("setpointRight: " + std::to_string(std::copysign(setpointRight, ArkLeng_right)));

				Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setpointLeft, ArkLeng_left) * -1);
				//}
			}

			if(Sright < ArkLeng_right - setpointRight){
				//accelerate Right{
				if(MaxVelocity <= VelocityPresentRight){
					VelocityPresentRight = MaxVelocity;
				}else{
					VelocityPresentRight += accelerationRight * difference;
				}

				setpointRight += VelocityPresentRight* difference;
				if(setpointRight > ArkLeng_right){
					setpointRight = ArkLeng_right;
				}

				frc::DriverStation::ReportWarning("encoderLeft: " + std::to_string(encoderLeft()));
				frc::DriverStation::ReportWarning("encoderRight: " + std::to_string(encoderRight()));
				//frc::DriverStation::ReportWarning("setpointLeft: " + std::to_string(std::copysign(setpointLeft, ArkLeng_left)));
				//frc::DriverStation::ReportWarning("setpointRight: " + std::to_string(std::copysign(setpointRight, ArkLeng_right)));

				Right_Back.Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setpointRight, ArkLeng_left));
				//}

			} else {
				//decelerate {
				VelocityPresentRight -= accelerationRight * difference;
				setpointRight += VelocityPresentRight * difference;
				if(setpointRight > ArkLeng_right){
					setpointRight = ArkLeng_right;
				}

				frc::DriverStation::ReportWarning("encoderLeft: " + std::to_string(encoderLeft()));
				frc::DriverStation::ReportWarning("encoderRight: " + std::to_string(encoderRight()));
				//frc::DriverStation::ReportWarning("setpointLeft: " + std::to_string(std::copysign(setpointLeft, ArkLeng_left)));
				//frc::DriverStation::ReportWarning("setpointRight: " + std::to_string(std::copysign(setpointRight, ArkLeng_right)));

				Right_Back.Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setpointRight, ArkLeng_left));
				//}
			}
		}
}
//----------------------------------------------------------------------------------------------
void Drive::Point(int angle, double sensitivity, double deadzone){
	while(lround(gyro.GetAngle()) % 360 < angle - (deadzone/2)){
		Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -sensitivity);
		Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -sensitivity);
	}
	while(lround(gyro.GetAngle()) % 360 > angle + (deadzone/2)){
		Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, sensitivity);
		Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, sensitivity);
	}
}
//----------------------------------------------------------------------------------------------
//Getter function
double Drive::encoder(){
	//average encoder value
	return (Left_Front.GetSelectedSensorPosition(0) + Right_Back.GetSelectedSensorPosition(0)) / 2;
}

double Drive::encoderLeft(){
	return Left_Front.GetSelectedSensorPosition(0);
}

double Drive::encoderRight(){
	return Right_Back.GetSelectedSensorPosition(0);
}

