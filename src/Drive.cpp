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

	//zero the encoders
	PIDdisable();
	Left_Front.SetSelectedSensorPosition(0, 0, 0);
	Right_Back.SetSelectedSensorPosition(0, 0, 0);
	Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0);
	Right_Back.Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0);
	PIDenable(P,I,D,F);

	//variables
	double setpoint;
	double lasttime;
	double curtime;
	double difference;
	double s; //displacement

	//set variables
	setpoint = 0;
	lasttime = counter.GetFPGATimestamp();

	while(setpoint < Dtot /*&& counter.GetFPGATimestamp()*1000 > timeout*/){
		s = ((Vf_at_end * Vf_at_end) - (VelocityPresent * VelocityPresent))/(2 * -acceleration);
		if(s < Dtot - setpoint){
			//accelerate {
			curtime = counter.GetFPGATimestamp();
			difference =  curtime - lasttime;
			lasttime = curtime;
			if(MaxVelocity <= VelocityPresent){
				setpoint += VelocityPresent * difference;
			}else{
				VelocityPresent += acceleration * difference;
				setpoint += VelocityPresent * difference;
			}
			if(setpoint > Dtot){
				setpoint = Dtot;
			}

			frc::DriverStation::ReportWarning("encoderLeft: " + std::to_string(encoderLeft()));
			frc::DriverStation::ReportWarning("encoderRight: " + std::to_string(encoderRight()));

			Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setpoint, Dtot) * -1);
			Right_Back.Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setpoint, Dtot));
			//}
		} else {
			//decelerate {
			curtime = counter.GetFPGATimestamp();
			difference =  curtime - lasttime;
			lasttime = curtime;
			VelocityPresent -= acceleration* difference;
			setpoint += VelocityPresent * difference;
			if(setpoint > Dtot){
				setpoint = Dtot;
			}

			frc::DriverStation::ReportWarning("encoderLeft: " + std::to_string(encoderLeft()));
			frc::DriverStation::ReportWarning("encoderRight: " + std::to_string(encoderRight()));

			Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setpoint, Dtot) * -1);
			Right_Back.Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setpoint, Dtot));
			//}
		}
	}
}

void Drive::PIDTurn(double Vf_at_end, double CoW, double acceleration, int timeout,double MaxVelocity, double a_left, double b_left, double a_right, double b_right, int angle){
/*
	//a_left, b_left, a_right, b_right in inches
	//a/b is from the equation of an elipse
	//angle in degrees
	//Dtot = total distance in inches
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
	acceleration = acceleration * 12 * ticksPERin;
	Vf_at_end = Vf_at_end * 12 * ticksPERin;
	MaxVelocity = MaxVelocity * 12 * ticksPERin;

	//variables
	double setpoint;
	double Sleft;
	double Sright;
	double lastime;
	double Left_Start;
	double Right_Start;
	//ark length of the left and right wheel
	double ArkLeng_left;
	double ArkLeng_right;
	double difference;

	//set variables
		//ark length of an eclipse
	ArkLeng_left = 2.0 * 3.14 * sqrt((std::pow(a_left, 2.0) + std::pow(b_left, 2.0))/2) * (angle/360);
	ArkLeng_right = 2.0 * 3.14 * sqrt((std::pow(a_right, 2.0) + std::pow(b_right, 2.0))/2) * (angle/360);
	Left_Start = encoderLeft();
	Right_Start = encoderRight();
	lastime = counter.GetFPGATimestamp();

	if(ArkLeng_left > ArkLeng_right){
		while(encoderLeft() - Left_Start < ArkLeng_left){
			Sleft = ((Vf_at_end * Vf_at_end) - (VpLeft() * VpLeft())/(2 * -acceleration));
			Sright =((Vf_at_end * Vf_at_end) - (VpRight() * VpRight())/(2 * -acceleration));
			if(Sleft < ArkLeng_left - (encoderLeft() - Left_Start)){
				//accelerate Left{
				if(MaxVelocity <= VpLeft()){

				}
				difference = counter.GetFPGATimestamp() - lastime;
				setpoint = encoderLeft() + (VpLeft() + acceleration*(difference)) * (difference);
				Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
				//}
				if(Sright < ArkLeng_right - (encoderRight() - Right_Start)){
					//accelerate Right {
					difference = counter.GetFPGATimestamp() - lastime;
					setpoint = encoderRight() + (VpRight() + acceleration*(difference)) * (difference);
					Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
					//}
				}
			} else {
				//decelerate Left & Right {
				difference = counter.GetFPGATimestamp() - lastime;
				setpoint = encoderLeft() + (VpLeft() - acceleration*(difference)) * (difference);
				Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
				setpoint = encoderRight() + (VpRight() - acceleration*(difference)) * (difference);
				Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
				//}
			}
		}
	}else{
		while (encoderRight() - Right_Start < ArkLeng_right){
			Sleft = ((Vf_at_end * Vf_at_end) - (VpLeft() * VpLeft())/(2 * -acceleration));
			Sright =((Vf_at_end * Vf_at_end) - (VpRight() * VpRight())/(2 * -acceleration));
			if(Sright < ArkLeng_right - (encoderRight() - Right_Start)){
				//accelerate Right {
				difference = counter.GetFPGATimestamp() - lastime;
				setpoint = encoderRight() + (VpRight() + acceleration*(difference)) * (difference);
				Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
				//}
				if(Sleft < ArkLeng_left - ( Left_Front.GetSelectedSensorPosition(0) - Left_Start)){
					//accelerate Left {
					difference = counter.GetFPGATimestamp() - lastime;
					setpoint = encoderLeft() + (VpLeft() + acceleration*(difference)) * (difference);
					Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
					//}
				} else {
					//decelerate Left {
					difference = counter.GetFPGATimestamp() - lastime;
					setpoint = encoderLeft() + (VpLeft() - acceleration*(difference)) * (difference);
					Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
					//}
				}
			} else {
				//decelerate Left & Right {
				difference = counter.GetFPGATimestamp() - lastime;
				setpoint = encoderLeft() + (VpLeft() - acceleration*(difference)) * (difference);
				Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
				setpoint = encoderRight() + (VpRight() - acceleration*(difference)) * (difference);
				Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
				//}
			}
		}
	}
	double temp_encoder = encoder();
	Left_Front.SetSelectedSensorPosition(temp_encoder,0,0);
	Right_Back.SetSelectedSensorPosition(temp_encoder,0,0);

*/
}

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

