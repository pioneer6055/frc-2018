/*
*   INPUTS
*   JoystickDrive   X axis = drive strafe
*   JoystickDrive   Y axis = drive fwd/rev
*
*   JoystickPlay    X axis = lift up/down
*   JoystickPlay    Y axis = arm up/down
*   JoystickPlay    4 button = grip intakes
*   JoystickPlay    5 button = grip ejects
*
*/
#include "Robot.h"

void Robot::RobotInit()
{
	SetPeriod(0.02);
	StickDrive = new Joystick(0); //USB
	StickPlay = new Joystick(1);  //USB
	MotorRF = new WPI_TalonSRX(1); //CAN
	MotorRR = new WPI_TalonSRX(2); //CAN
	MotorLR = new WPI_TalonSRX(3); //CAN
	MotorLF = new WPI_TalonSRX(4); //CAN
	MotorLift = new VictorSP(0); //PWM  - Need 3 Splitters
	MotorArm = new VictorSP(1);  //PWM
	PotArm = new AnalogPotentiometer(0,12,0); //AI
	MotorGrip = new VictorSP(2); //PWM - Need Splitter
	DriveTrain = new DifferentialDrive(*MotorLF,*MotorRF);
	LimitLiftHi = new DigitalInput(0); //DI
	LimitLiftLo = new DigitalInput(1); //DI
	LimitGripStop = new DigitalInput(2); //DI   - not connected
	ThumbWheel_1 = new DigitalInput(10); //DI on NAVX
	ThumbWheel_2 = new DigitalInput(11); //DI on NAVX
	ThumbWheel_4 = new DigitalInput(12); //DI on NAVX
	ThumbWheel_8 = new DigitalInput(13); //DI on NAVX
	AutoProfile = new Profile();
	ElapsedTimer = new Timer();
	ElapsedTimer->Start();
	AutoTimer = new Timer();
	AutoTimer->Start();
	DriveTrain->SetSafetyEnabled(false);
	MotorRF->SetSafetyEnabled(false);
	MotorLF->SetSafetyEnabled(false);
	MotorLR->SetSafetyEnabled(false);
	MotorRR->SetSafetyEnabled(false);
	MotorLift->SetSafetyEnabled(false);
	MotorArm->SetSafetyEnabled(false);
	MotorGrip->SetSafetyEnabled(false);
	MotorLF->SetInverted(true);
	MotorLR->SetInverted(true);
	MotorRF->SetInverted(true);
	MotorRR->SetInverted(true);
	MotorArm->SetInverted(false);
	MotorLift->SetInverted(false);
	MotorGrip->SetInverted(false);

	MotorLF->SetNeutralMode(NeutralMode::Brake);
	MotorRF->SetNeutralMode(NeutralMode::Brake);
	MotorLR->SetNeutralMode(NeutralMode::Brake);
	MotorRR->SetNeutralMode(NeutralMode::Brake);

	//SetRampRate(1.0);
	/*MotorLF->ConfigPeakCurrentLimit(35, 10);
	MotorLF->ConfigPeakCurrentDuration(200, 10);
	MotorLF->ConfigContinuousCurrentLimit(30, 10);
	MotorLF->EnableCurrentLimit(true);

	MotorRF->ConfigPeakCurrentLimit(35, 10);
	MotorRF->ConfigPeakCurrentDuration(200, 10);
	MotorRF->ConfigContinuousCurrentLimit(30, 10);
	MotorRF->EnableCurrentLimit(true);

	MotorLR->ConfigPeakCurrentLimit(35, 10);
	MotorLR->ConfigPeakCurrentDuration(200, 10);
	MotorLR->ConfigContinuousCurrentLimit(30, 10);
	MotorLR->EnableCurrentLimit(true);

	MotorRR->ConfigPeakCurrentLimit(35, 10);
	MotorRR->ConfigPeakCurrentDuration(200, 10);
	MotorRR->ConfigContinuousCurrentLimit(30, 10);
	MotorRR->EnableCurrentLimit(true); */

	MotorLR->Follow(*MotorLF);
	MotorRR->Follow(*MotorRF);

	MotorLF->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,0);
	MotorLF->SetSensorPhase(false);
	MotorRF->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,0);
	MotorRF->SetSensorPhase(false);

	try
	{
		Gyro = new AHRS(SPI::Port::kMXP,60);
		printf("NAVX Initialized OK\n");
	}
	catch (std::exception& ex )
	{
		std::string err_string = "Error initializing navX:  ";
		err_string += ex.what();
		DriverStation::ReportError(err_string.c_str());
	}
	//camera = CameraServer::GetInstance()->StartAutomaticCapture();
}

void Robot::AutonomousInit()
{
	AutoState = 0;
	//set min/max speed range for forward/backward moves
	AutoProfile->ProfileMinSpeed = 0.35;
	AutoProfile->ProfileMaxSpeed = 0.75;
	//don't slow down between continuous movements
	AutoProfile->ProfileContinuous = false;
	AutoProfile->Initialize();
	//find out assignments for switch and plate from FMS
	GameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	ThumbWheel = GetThumbWheel();  //determines which autonomous profile to run
	ZeroHeading();
	//zero the encoders
	MotorLF->SetSelectedSensorPosition(0,0,0);
	MotorRF->SetSelectedSensorPosition(0,0,0);
	AutoTimer->Reset();
}

void Robot::AutonomousPeriodic()
{
	switch(ThumbWheel)
	{
		case 1:
			//drive straight to cross line
			Auto_Straight();
			break;
		case 2:
			//if DATA = LXX place switch left else place switch right
			Auto_SwitchFrom2();
			break;
		case 3:
			//if DATA = LXX place switch else if DATA=XLX place scale else cross line
			Auto_SwitchOrScaleFrom1();
			break;
		case 4:
			//if DATA = RXX place switch else if DATA=XRX place scale else cross line
			Auto_SwitchOrScaleFrom3();
			break;
		case 5:
			//if DATA = XLX place scale else if DATA=LXX place switch else cross line
			Auto_ScaleOrSwitchFrom1();
			break;
		case 6:
			//if DATA = XRX place scale else if DATA=RXX place switch else cross line
			Auto_ScaleOrSwitchFrom3();
			break;
		default:
			DriveTrain->ArcadeDrive(0.0,0.0);
			MotorArm->Set(0.0);
			MotorLift->Set(0.0);
			MotorGrip->Set(0.0);
			break;
	}
}

void Robot::TeleopInit()
{
	MotorLF->SetSelectedSensorPosition(0,0,0);
	MotorRF->SetSelectedSensorPosition(0,0,0);
	ElapsedTimer->Reset();
}

void Robot::TeleopPeriodic()
{
	double stickDriveX = StickDrive->GetRawAxis(0);
	double stickDriveY = StickDrive->GetRawAxis(1);
	double stickPlayX = StickPlay->GetRawAxis(0);
	double stickPlayY = StickPlay->GetRawAxis(1);
	double gripSpeedFactor = fabs(((StickPlay->GetRawAxis(3) * -1)+1.0f))/2.0f;
	double posArm = PotArm->Get();

	//drive via single joystick
	if(fabs(stickDriveX) > 0.15 || fabs(stickDriveY) > 0.15)
	{
		DriveTrain->ArcadeDrive(stickDriveY,stickDriveX *-1,false);
	}
	else DriveTrain->StopMotor();

	//Run the lift and arm
	if(fabs(stickPlayX) > 0.25)
	{
		if (stickPlayX > 0) stickPlayX -= 0.25;
		else stickPlayX += 0.25;
		MotorLift->Set(GetLiftSpeed(stickPlayX,!LimitLiftLo->Get(),!LimitLiftHi->Get()));
	}
	else
	{
		MotorLift->StopMotor();
	}

	if(fabs(stickPlayY) > 0.25)
	{
		if (stickPlayY > 0) stickPlayY -= 0.25;
		else stickPlayY += 0.25;
		MotorArm->Set(GetArmSpeed(stickPlayY,posArm,7.5,1.5,1.0,0.0));
	}
	else
	{
		MotorArm->StopMotor();
	}

	//Run the Gripper
	MotorGrip->Set(GetGripSpeed(StickPlay->GetRawButton(4),StickPlay->GetRawButton(5),gripSpeedFactor));

	//Show debug info
	if(ElapsedTimer->HasPeriodPassed(1.0))
	{
		ElapsedTimer->Reset();
		printf("ArmPos= %.1f LiftLO=%d LiftHI=%d Yaw=%f.1 Dist=%f.1\n",posArm,LimitLiftLo->Get(),LimitLiftHi->Get(),GetHeading(),GetDistance());
	}
}

void Robot::DisabledPeriodic()
{
	MotorLF->SetNeutralMode(NeutralMode::Brake);
	MotorRF->SetNeutralMode(NeutralMode::Brake);
	MotorLR->SetNeutralMode(NeutralMode::Brake);
	MotorRR->SetNeutralMode(NeutralMode::Brake);
}

double Robot::GetArmSpeed(double stickY, double pos, double pMax, double pMin, double sMax, double sMin)
{
	if((stickY < -0.15 && pos > pMin) || (stickY > 0.15 && pos < pMax))
	{
		double spdFactor = sMax;
		if(pos >= 10) spdFactor = (12 - pos)/2;
		if(pos <= 2) spdFactor = pos/2;
		if(spdFactor > sMax) spdFactor = sMax;
		if(spdFactor < sMin) spdFactor = sMin;
		return stickY * spdFactor;
	}
	else return 0.0;
}

double Robot::GetLiftSpeed(double stickX, bool limitLiftLo, bool limitLiftHi)
{
	if((stickX < 0 && !limitLiftHi) || (stickX > 0 && !limitLiftLo)) return stickX;
	else return 0.0;
}

double Robot::GetGripSpeed(bool butIntake, bool butReject, double speedFactor)
{
	double ret = 0.0;

	if(butIntake && !butReject) ret = -1.0 * speedFactor;
	if(butReject && !butIntake) ret = 1.0 * speedFactor;
	return ret;
}

void Robot::SetRampRate(double secs)
{
	MotorLF->ConfigOpenloopRamp(secs,10);
	MotorRF->ConfigOpenloopRamp(secs,10);
	MotorLR->ConfigOpenloopRamp(secs,10);
	MotorRR->ConfigOpenloopRamp(secs,10);
}

double Robot::GetHeading()
{
	double offsetYaw = AutoProfile->GetNormalizedHeading(Gyro->GetYaw()) - HeadingOffset;
	if(offsetYaw < 0) offsetYaw += 360;
	return offsetYaw;
}

void Robot::ZeroHeading()
{
	HeadingOffset = AutoProfile->GetNormalizedHeading(Gyro->GetYaw());
}

double Robot::GetDistance()
{
	return MotorRF->GetSelectedSensorPosition(0) * mag_FeetPerPulse;
	//return MotorRF->GetSensorCollection().GetQuadraturePosition() * mag_FeetPerPulse;
}

int Robot::GetThumbWheel()
{
	bool d1 = ThumbWheel_1->Get();
	bool d2 = ThumbWheel_2->Get();
	bool d4 = ThumbWheel_4->Get();
	bool d8 = ThumbWheel_8->Get();

	int ret = 0;
	if(!d1 && d2 && d4 && !d8) ret = 9;
	if(d1 && d2 && d4 && !d8) ret = 8;
	if(!d1 && !d2 && !d4 && d8) ret = 7;
	if(d1 && !d2 && !d4 && d8) ret = 6;
	if(!d1 && d2 && !d4 && d8) ret = 5;
	if(d1 && d2 && !d4 && d8) ret = 4;
	if(!d1 && !d2 && d4 && d8) ret = 3;
	if(d1 && !d2 && d4 && d8) ret = 2;
	if(!d1 && d2 && d4 && d8) ret = 1;
	printf("ThumbWheel= %d\n",ret);
	return ret;
}

double ffilter(double raw, double current, double lpf)
{
	double weight = std::fmin(0.99999,std::fmax(lpf,0));
	return ((1-weight) * raw) + (weight * current);
}


START_ROBOT_CLASS(Robot)
