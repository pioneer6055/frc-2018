/*
 * Robot.h
 *
 *  Created on: Jan 29, 2018
 *      Author: a851729
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include "AHRS.h"
#include "Profile.h"
#include "ctre/Phoenix.h"
#include "WPILib.h"

class Robot : public frc::TimedRobot
{
private:
	Joystick *StickDrive;
	Joystick *StickPlay;
	WPI_TalonSRX *MotorLF;
	WPI_TalonSRX *MotorRF;
	WPI_TalonSRX *MotorLR;
	WPI_TalonSRX *MotorRR;
	VictorSP *MotorLift;
	VictorSP *MotorArm;
	AnalogPotentiometer *PotArm;
	VictorSP *MotorGrip;
	DifferentialDrive *DriveTrain;
	DigitalInput *LimitLiftHi;
	DigitalInput *LimitLiftLo;
	DigitalInput *LimitGripStop;
	DigitalInput *ThumbWheel_1;
	DigitalInput *ThumbWheel_2;
	DigitalInput *ThumbWheel_4;
	DigitalInput *ThumbWheel_8;
	Profile *AutoProfile;
	AHRS *Gyro;
	Timer *ElapsedTimer;
	Timer *AutoTimer;
	//cs::UsbCamera camera;
	float HeadingOffset = 0.0f;
	int AutoState = 0;
	int ThumbWheel = 0;
	std::string GameData;
	// DistancePerPulse = (1/PulsesPerRevolution) * PI * WheelDiameter
	//for CTRE Mag Encoder = 4096 PPR?
	//(1/1024) * 3.1415 * 0.33333  for 4 inch wheel = 0.00102265
	//(1/4096) * 3.1415 * 0.33333  for 4 inch wheel = 0.00025566
	//(1/1024) * 3.1415 * 0.5   for 6 inch wheel = 0.00153398
	//(1/4096) * 3.1415 * 0.5   for 6 inch wheel = 0.000383495
	//float mag_FeetPerPulse = 0.0004635593; //what we were using for 4 inch wheel
	float mag_FeetPerPulse = 0.0008538755; //0.000383495;
	float wheel_circumference = 1.57079632679; //6 inch wheel
	double TurnMaxSpeed = 0.5;
public:

	void RobotInit();
	void AutonomousInit();
	void AutonomousPeriodic();
	void TeleopInit();
	void TeleopPeriodic();
	void DisabledPeriodic();
	double ffilter(double raw, double current, double lpf);
	double GetHeading();
	void ZeroHeading();
	double GetDistance();
	int GetThumbWheel();
	double GetArmSpeed(double stickY, double pos, double pMax, double pMin, double sMax, double sMin);
	double GetLiftSpeed(double stickX, bool limitLo, bool limitHi);
	double GetGripSpeed(bool butIntake, bool butReject, double speedFactor);
	void SetRampRate(double secs);

	void ExecuteProfile();
	void Auto_Drive(double outputMagnitude, double curve);
	double Clamp(double value, double min, double max);
	void Auto_Straight();
	void Auto_SwitchFrom2();
	void Auto_SwitchOrScaleFrom1();
	void Auto_SwitchOrScaleFrom3();
	void Auto_ScaleOrSwitchFrom1();
	void Auto_ScaleOrSwitchFrom3();
	bool EjectCrate(double seconds, double speed);
	bool LiftRaisedToUpperLimit();
	bool ArmLowered(double height);
	bool LiftRaisedToUpperLimitAndArmLowered(double height);
};



#endif /* SRC_ROBOT_H_ */
