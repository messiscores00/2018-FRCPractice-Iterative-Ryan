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

void Drive::ArcadeDrive(double deadzone, double xSpeed, double zRotation, bool squaredInputs){
	double leftMotorOutput;
	double rightMotorOutput;

	if (fabs(xSpeed) <= deadzone){
	    xSpeed = 0;
	}

	if (squaredInputs) {
	   xSpeed = std::copysign(xSpeed * xSpeed, xSpeed);
	   zRotation = std::copysign(zRotation * zRotation, zRotation);
	}

	   double maxInput = std::copysign(std::max(std::abs(xSpeed), std::abs(zRotation)), xSpeed);

	   if (xSpeed >= 0.0)
	   {
	      // First quadrant, else second quadrant
	      if (zRotation >= 0.0)
	      {
	         leftMotorOutput = maxInput;
	         rightMotorOutput = xSpeed - zRotation;
	      }
	      else
	      {
	         leftMotorOutput = xSpeed + zRotation;
	         rightMotorOutput = maxInput;
	      }
	   }
	   else
	   {
	      // Third quadrant, else fourth quadrant
	      if (zRotation >= 0.0)
	      {
	         leftMotorOutput = xSpeed + zRotation;
	         rightMotorOutput = maxInput;
	      }
	      else
	      {
	         leftMotorOutput = maxInput;
	         rightMotorOutput = xSpeed - zRotation;
	      }
	   }
	   Left_Front.Set(leftMotorOutput);
	   Right_Front.Set(-rightMotorOutput);

	   frc::DriverStation::ReportError("xSpeed: " + std::to_string(xSpeed));
	   frc::DriverStation::ReportError("zRotation: " + std::to_string(zRotation));
}

void Drive::PIDMove(double Dtot, double Vf_at_end, double CoW, double acceleration, int timeout){
	//Dtot = total distance in inches
	//Vf_at_end = final velocity in ft/sec
	//CoW = circumference of Wheel in inches
	//acceleration = max deceleration in ft/s^2 note: do not make this negative
	//timeout in milliseconds

	//convert variables into ticks
	Dtot = Dtot * (4096/CoW);
	acceleration = acceleration * 12 * (4096/CoW);
	Vf_at_end = Vf_at_end * 12 * (4096/CoW);

	//variables
	double setpoint;
	double lasttime;
	double difference;
	double s; //displacement
	double start;

	//set variables
	start = encoder();
//	lasttime = counter.GetFPGATimestamp();

	while(encoder() - start < Dtot /*&& counter.GetFPGATimestamp()*1000 > timeout*/){
		frc::DriverStation::ReportWarning("EncoderLeft: " + std::to_string(encoderLeft()));
		frc::DriverStation::ReportWarning("EncoderRight: " + std::to_string(encoderRight()));
		lasttime = counter.GetFPGATimestamp();
		s = ((Vf_at_end * Vf_at_end) - (Vp() * Vp())/(2 * -acceleration));
			if(s < Dtot - (encoder() - start)){
				//accelerate {
				//todo: if we reach MaxV then don't accelerate
				frc::DriverStation::ReportWarning("accelerate ");
				difference = counter.GetFPGATimestamp() - lasttime;
				setpoint = encoder() + (Vp() + acceleration*(difference)) * (difference);
				Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, -setpoint);
				Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
				//}
			} else {
				//decelerate {
				frc::DriverStation::ReportWarning("decelerate ");
				difference = counter.GetFPGATimestamp() - lasttime;
				setpoint = encoder() + (Vp() - acceleration*(difference)) * (difference);
				Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, -setpoint);
				Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
				//}
			}
		}
}

void Drive::PIDTurn(double Vf_at_end, double CoW, double Maxacceleration, int timeout, double sensitivity, double a_left, double b_left, double a_right, double b_right, int angle){
	//converting variables to ticks
	sensitivity = sensitivity * 12 * (4096/CoW);
	Vf_at_end = Vf_at_end * 12 * (4096/CoW);
	a_left = a_left * 12 * (4096/CoW);
	b_left = b_left * 12 * (4096/CoW);
	a_right = a_right * 12 * (4096/CoW);
	b_right = b_right * 12 * (4096/CoW);

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
				Sleft = VpLeft() * (2.0 * ArkLeng_left/(VpLeft()+Vf_at_end)) + ((-Maxacceleration*2*std::pow(ArkLeng_left, 2.0))/VpLeft() + std::pow(Vf_at_end, 2.0));
				Sright = VpRight() * (2.0 * ArkLeng_right/(VpRight()+Vf_at_end)) + ((-Maxacceleration*2*std::pow(ArkLeng_right, 2.0))/VpRight() + std::pow(Vf_at_end, 2.0));
				if(Sleft < ArkLeng_left - (encoderLeft() - Left_Start)){
					//accelerate Left{
					difference = counter.GetFPGATimestamp() - lastime;
					setpoint = encoderLeft() + (VpLeft() + Maxacceleration*(difference)) * (difference);
					Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
					//}
					if(Sright < ArkLeng_right - (encoderRight() - Right_Start)){
						//accelerate Right {
						difference = counter.GetFPGATimestamp() - lastime;
						setpoint = encoderRight() + (VpRight() + Maxacceleration*(difference)) * (difference);
						Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
						//}
					}
				} else {
					//decelerate Left & Right {
					difference = counter.GetFPGATimestamp() - lastime;
					setpoint = encoderLeft() + (VpLeft() - Maxacceleration*(difference)) * (difference);
					Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
					setpoint = encoderRight() + (VpRight() - Maxacceleration*(difference)) * (difference);
					Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
					//}
				}

			}
		}else{
			while (encoderRight() - Right_Start < ArkLeng_right){
				Sleft = VpLeft() * (2.0 * ArkLeng_left/(VpLeft()+Vf_at_end)) + ((-Maxacceleration*2*std::pow(ArkLeng_left, 2.0))/VpLeft() + std::pow(Vf_at_end, 2.0));
				Sright = VpRight() * (2.0 * ArkLeng_right/(VpRight()+Vf_at_end)) + ((-Maxacceleration*2*std::pow(ArkLeng_right, 2.0))/VpRight() + std::pow(Vf_at_end, 2.0));
				if(Sright < ArkLeng_right - (encoderRight() - Right_Start)){
					//accelerate Right {
					difference = counter.GetFPGATimestamp() - lastime;
					setpoint = encoderRight() + (VpRight() + Maxacceleration*(difference)) * (difference);
					Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
					//}
					if(Sleft < ArkLeng_left - ( Left_Front.GetSelectedSensorPosition(0) - Left_Start)){
						//accelerate Left {
						difference = counter.GetFPGATimestamp() - lastime;
						setpoint = encoderLeft() + (VpLeft() + Maxacceleration*(difference)) * (difference);
						Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
						//}
					} else {
						//decelerate Left {
						difference = counter.GetFPGATimestamp() - lastime;
						setpoint = encoderLeft() + (VpLeft() - Maxacceleration*(difference)) * (difference);
						Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
						//}
					}
				} else {
					//decelerate Left & Right {
					difference = counter.GetFPGATimestamp() - lastime;
					setpoint = encoderLeft() + (VpLeft() - Maxacceleration*(difference)) * (difference);
					Left_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
					setpoint = encoderRight() + (VpRight() - Maxacceleration*(difference)) * (difference);
					Right_Front.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
					//}
				}
			}
		}
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

double Drive::Vp(){
	//average Present Velocity
	return (Left_Front.GetSelectedSensorVelocity(0) + Right_Back.GetSelectedSensorVelocity(0)) / 2;
}

double Drive::VpLeft(){
	return Left_Front.GetSelectedSensorVelocity(0);
}

double Drive::VpRight(){
	return Right_Back.GetSelectedSensorVelocity(0);
}
