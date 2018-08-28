/*
 * Autonomous.cpp
 *
 *  Created on: Jan 29, 2018
 *      Author: a851729
 */

#include "robot.h"

void Robot::ExecuteProfile()
{

	AutoProfile->ExecuteProfile(GetHeading(),fabs(GetDistance()));
	Auto_Drive(AutoProfile->OutputMagnitude,AutoProfile->Curve);
}

/**
 * Drive the motors at "outputMagnitude" and "curve".
 * Both outputMagnitude and curve are -1.0 to +1.0 values, where 0.0 represents
 * stopped and not turning. curve < 0 will turn left and curve > 0 will turn
 * right.
 *
 * The algorithm for steering provides a constant turn radius for any normal
 * speed range, both forward and backward. Increasing m_sensitivity causes
 * sharper turns for fixed values of curve.
 *
 * This function will most likely be used in an autonomous routine.
 *
 * @param outputMagnitude The speed setting for the outside wheel in a turn,
 *                        forward or backwards, +1 to -1.
 * @param curve           The rate of turn, constant for different forward
 *                        speeds. Set curve < 0 for left turn or curve > 0 for
 *                        right turn.
 *
 * Set curve = e^(-r/w) to get a turn radius r for wheelbase w of your robot.
 * Conversely, turn radius r = -ln(curve)*w for a given value of curve and
 * wheelbase w.
 */

void Robot::Auto_Drive(double outputMagnitude, double curve)
{
	double m_sensitivity = 0.75;
	double leftOutput, rightOutput;
	if (curve < 0)
	{
		double value = std::log(-curve);
		double ratio = (value - m_sensitivity) / (value + m_sensitivity);
		if (ratio == 0) ratio = .0000000001;
		leftOutput = outputMagnitude / ratio;
		rightOutput = outputMagnitude;
	}
	else if (curve > 0)
	{
		double value = std::log(curve);
		double ratio = (value - m_sensitivity) / (value + m_sensitivity);
		if (ratio == 0) ratio = .0000000001;
		leftOutput = outputMagnitude;
		rightOutput = outputMagnitude / ratio;
	}
	else
	{
		leftOutput = outputMagnitude;
		rightOutput = outputMagnitude;
	}
	MotorLF->Set(Clamp(leftOutput,-1.0,1.0));
	MotorRF->Set(-Clamp(rightOutput,-1.0,1.0));
}

double Robot::Clamp(double value, double min, double max)
{
	if(value > max){ return max;}
	if(value < min){ return min;}
	return value;
}

void Robot::Auto_Straight()
{
	switch(AutoState)  //autonomous sequencer
	{
		case 0:
			AutoProfile->Initialize();
			AutoProfile->ClearProfile();
			AutoProfile->AddMove(AutoProfile->kProfileForward,8);
			AutoProfile->ProfileLoaded = true;
			AutoState++;
			break;
		case 1:
			if(AutoProfile->ProfileLoaded && !AutoProfile->ProfileCompleted) ExecuteProfile();
			else
			{
				AutoState++;
				printf("Auto_Straight Completed\n");
			}
			break;
		default:
			DriveTrain->ArcadeDrive(0.0,0.0);
			MotorArm->Set(0.0);
			MotorLift->Set(0.0);
			MotorGrip->Set(0.0);
			break;
	}
}

void Robot::Auto_SwitchFrom2()   //left wheels on center line
{
	switch(AutoState)  //autonomous sequencer
	{
		case 0:
			AutoProfile->Initialize();
			AutoProfile->ClearProfile();
			AutoProfile->AddMove(AutoProfile->kProfileForward,2.5);
			if(GameData[0] == 'L') //deliver to left switch plate
			{
				AutoProfile->AddTurn(315,-TurnMaxSpeed);
				AutoProfile->AddMove(AutoProfile->kProfileForward,6.3);
				AutoProfile->AddTurn(0,TurnMaxSpeed);
			}
			else  //deliver to right switch plate
			{
				AutoProfile->AddTurn(35,TurnMaxSpeed);
				AutoProfile->AddMove(AutoProfile->kProfileForward,5.5);
				AutoProfile->AddTurn(0,-TurnMaxSpeed);
			}
			AutoProfile->ProfileLoaded = true;
			AutoState++;
			break;
		case 1:
			if(AutoProfile->ProfileLoaded && !AutoProfile->ProfileCompleted) ExecuteProfile();
			else AutoState++;
			break;
		case 2:  //lower arm
			if(ArmLowered(4.0))
			{
				AutoState++;
				AutoTimer->Reset();
			}
			break;
		case 3:  //spit out crate
			if(EjectCrate(2.0,0.35))
			{
				AutoState++;
				printf("SwitchFrom2 Completed\n");
			}
			break;
		default:
			DriveTrain->ArcadeDrive(0.0,0.0);
			MotorArm->Set(0.0);
			MotorLift->Set(0.0);
			MotorGrip->Set(0.0);
			break;
	}
}

void Robot::Auto_SwitchOrScaleFrom1()    //start right wheels 9.5 ft left of center line
{
	static int choice = 0;
	double spd = 0.35;
	switch(AutoState)  //autonomous sequencer
	{
		case 0:
			AutoProfile->Initialize();
			AutoProfile->ClearProfile();
			//deliver to left switch plate
			if(GameData[0] == 'L') choice = 1;
			//deliver to left scale
			if(choice != 1 && GameData[1] == 'L') choice = 2;
			switch(choice)
			{
				case 0:
					AutoProfile->AddMove(AutoProfile->kProfileForward,8);
					break;
				case 1:
					AutoProfile->AddMove(AutoProfile->kProfileForward,13);
					AutoProfile->AddTurn(90,TurnMaxSpeed);
					AutoProfile->AddMove(AutoProfile->kProfileForward,2);
					printf("SWITCH Chosen\n");
					break;
				case 2:
					AutoProfile->AddMove(AutoProfile->kProfileForward,44);
					AutoProfile->AddTurn(90,TurnMaxSpeed);
					//AutoProfile->AddMove(AutoProfile->kProfileForward,2);
					printf("SCALE Chosen\n");
					break;
			}
			AutoProfile->ProfileLoaded = true;
			AutoState++;
			break;
		case 1:  //move to target
			if(AutoProfile->ProfileLoaded && !AutoProfile->ProfileCompleted) ExecuteProfile();
			else
			{
				AutoState++;
				AutoTimer->Reset();
			}
			break;
		case 2:  //raise lift if at scale - lower arm if at switch
			switch(choice)
			{
				case 1:
					if(ArmLowered(4.0))
					{
						AutoState++;
						AutoTimer->Reset();
					}
					break;
				case 2:
					if(LiftRaisedToUpperLimitAndArmLowered(6.0))
					{
						AutoState++;
						AutoTimer->Reset();
					}
					break;
				default:
					AutoState++;
					AutoTimer->Reset();
					break;
			}
			break;
		case 3:  //spit out crate
			if(choice > 1) spd = 1.0;
			if(EjectCrate(2.0,spd))
			{
				AutoState++;
				printf("SwitchOrScaleFrom1 Completed\n");
			}
			break;
		default:
			DriveTrain->ArcadeDrive(0.0,0.0);
			MotorArm->Set(0.0);
			MotorLift->Set(0.0);
			MotorGrip->Set(0.0);
			break;
	}
}

void Robot::Auto_SwitchOrScaleFrom3()    //start left wheels 9.5 ft right of center line
{
	static int choice = 0;
	double spd = 0.35;
	switch(AutoState)  //autonomous sequencer
	{
		case 0:
			AutoProfile->Initialize();
			AutoProfile->ClearProfile();
			//deliver to switch
			if(GameData[0] == 'R') choice = 1;
			//deliver to scale
			if(choice != 1 && GameData[1] == 'R') choice = 2;
			switch(choice)
			{
				case 0:
					AutoProfile->AddMove(AutoProfile->kProfileForward,8);
					break;
				case 1:
					AutoProfile->AddMove(AutoProfile->kProfileForward,13);
					AutoProfile->AddTurn(270,-TurnMaxSpeed);
					AutoProfile->AddMove(AutoProfile->kProfileForward,2);
					printf("SWITCH Chosen\n");
					break;
				case 2:
					AutoProfile->AddMove(AutoProfile->kProfileForward,44);
					AutoProfile->AddTurn(270,-TurnMaxSpeed);
					//AutoProfile->AddMove(AutoProfile->kProfileForward,2);
					printf("SCALE Chosen\n");
					break;
			}
			AutoProfile->ProfileLoaded = true;
			AutoState++;
			break;
		case 1:  //move to target
			if(AutoProfile->ProfileLoaded && !AutoProfile->ProfileCompleted) ExecuteProfile();
			else
			{
				AutoState++;
				AutoTimer->Reset();
			}
			break;
		case 2:  //raise lift if at scale - lower arm if at switch
			switch(choice)
			{
				case 1:
					if(ArmLowered(3.0))
					{
						AutoState++;
						AutoTimer->Reset();
					}
					break;
				case 2:
					if(LiftRaisedToUpperLimitAndArmLowered(6.0))
					{
						AutoState++;
						AutoTimer->Reset();
					}
					break;
				default:
					AutoState++;
					AutoTimer->Reset();
					break;
			}
			break;
		case 3:  //spit out crate
			if(choice > 1) spd = 1.0;
			if(EjectCrate(2.0,spd))
			{
				AutoState++;
				printf("SwitchOrScaleFrom3 Completed\n");
			}
			break;
		default:
			DriveTrain->ArcadeDrive(0.0,0.0);
			MotorArm->Set(0.0);
			MotorLift->Set(0.0);
			MotorGrip->Set(0.0);
			break;
	}
}

void Robot::Auto_ScaleOrSwitchFrom1()    //start right wheels 9.5 ft left of center line
{
	static int choice = 0;
	double spd = 0.35;
	switch(AutoState)  //autonomous sequencer
	{
		case 0:
			AutoProfile->Initialize();
			AutoProfile->ClearProfile();
			//deliver to scale
			if(GameData[1] == 'L') choice = 2;
			//deliver to switch
			if(choice != 2 && GameData[0] == 'L') choice = 1;
			switch(choice)
			{
				case 0:
					AutoProfile->AddMove(AutoProfile->kProfileForward,8);
					break;
				case 1:
					AutoProfile->AddMove(AutoProfile->kProfileForward,18);
					AutoProfile->AddTurn(90,TurnMaxSpeed);
					AutoProfile->AddMove(AutoProfile->kProfileForward,2.8);
					printf("SWITCH Chosen\n");
					break;
				case 2:
					AutoProfile->AddMove(AutoProfile->kProfileForward,41.3);
					AutoProfile->AddTurn(90,TurnMaxSpeed);
					//AutoProfile->AddMove(AutoProfile->kProfileForward,2);
					printf("SCALE Chosen\n");
					break;
			}
			AutoProfile->ProfileLoaded = true;
			AutoState++;
			break;
		case 1:  //move to target
			if(AutoProfile->ProfileLoaded && !AutoProfile->ProfileCompleted) ExecuteProfile();
			else
			{
				AutoState++;
				AutoTimer->Reset();
			}
			break;
		case 2:  //raise lift if at scale - lower arm if at switch
			switch(choice)
			{
				case 1:
					if(ArmLowered(4.0))
					{
						AutoState++;
						AutoTimer->Reset();
					}
					break;
				case 2:
					if(LiftRaisedToUpperLimitAndArmLowered(6.0))
					{
						AutoState++;
						AutoTimer->Reset();
					}
					break;
				default:
					AutoState++;
					AutoTimer->Reset();
					break;
			}
			break;
		case 3:  //spit out crate
			if(choice > 1) spd = 1.0;
			if(EjectCrate(2.0,spd))
			{
				AutoState++;
				printf("ScaleOrSwitchFrom1 Completed\n");
			}
			break;
		default:
			DriveTrain->ArcadeDrive(0.0,0.0);
			MotorArm->Set(0.0);
			MotorLift->Set(0.0);
			MotorGrip->Set(0.0);
			break;
	}
}

void Robot::Auto_ScaleOrSwitchFrom3()    //start left wheels 9.5 ft right of center line
{
	static int choice = 0;
	double spd = 0.35;
	switch(AutoState)  //autonomous sequencer
	{
		case 0:
			AutoProfile->Initialize();
			AutoProfile->ClearProfile();
			//deliver to scale
			if(GameData[1] == 'R') choice = 2;
			//deliver to switch
			if(choice != 2 && GameData[0] == 'R') choice = 1;
			switch(choice)
			{
				case 0:
					AutoProfile->AddMove(AutoProfile->kProfileForward,8);
					break;
				case 1:
					AutoProfile->AddMove(AutoProfile->kProfileForward,18);
					AutoProfile->AddTurn(270,-TurnMaxSpeed);
					AutoProfile->AddMove(AutoProfile->kProfileForward,2.8);
					printf("SWITCH Chosen\n");
					break;
				case 2:
					AutoProfile->AddMove(AutoProfile->kProfileForward,41.3);
					AutoProfile->AddTurn(270,-TurnMaxSpeed);
					//AutoProfile->AddMove(AutoProfile->kProfileForward,2);
					printf("SCALE Chosen\n");
					break;
			}
			AutoProfile->ProfileLoaded = true;
			AutoState++;
			break;
		case 1:  //move to target
			if(AutoProfile->ProfileLoaded && !AutoProfile->ProfileCompleted) ExecuteProfile();
			else
			{
				AutoState++;
				AutoTimer->Reset();
			}
			break;
		case 2:  //raise lift if at scale - lower arm if at switch
			switch(choice)
			{
				case 1:
					if(ArmLowered(4.0))
					{
						AutoState++;
						AutoTimer->Reset();
					}
					break;
				case 2:
					if(LiftRaisedToUpperLimitAndArmLowered(6.0))
					{
						AutoState++;
						AutoTimer->Reset();
					}
					break;
				default:
					AutoState++;
					AutoTimer->Reset();
					break;
			}
			break;
		case 3:  //spit out crate
			if(choice > 1) spd = 1.0;
			if(EjectCrate(2.0,spd))
			{
				AutoState++;
				printf("ScaleOrSwitchFrom3 Completed\n");
			}
			break;
		default:
			DriveTrain->ArcadeDrive(0.0,0.0);
			MotorArm->Set(0.0);
			MotorLift->Set(0.0);
			MotorGrip->Set(0.0);
			break;
	}
}

bool Robot::EjectCrate(double seconds, double speed)
{
	if(!AutoTimer->HasPeriodPassed(seconds))
	{
		MotorGrip->Set(fabs(speed));
		return false;
	}
	else
	{
		MotorGrip->Set(0.0);
		return true;
	}
}

bool Robot::LiftRaisedToUpperLimit()
{
	if(LimitLiftHi->Get())
	{
		MotorLift->Set(GetLiftSpeed(-0.75,!LimitLiftLo->Get(),!LimitLiftHi->Get()));
		return false;
	}
	else
	{
		MotorLift->Set(0.0);
		return true;
	}
}

bool Robot::ArmLowered(double height)
{
	double posArm = PotArm->Get();
	if(posArm > height)
	{
		MotorArm->Set(GetArmSpeed(-0.5,posArm,7.5,1.5,1.0,0.0));
		return false;
	}
	else
	{
		MotorArm->Set(0.0);
		return true;
	}
}

bool Robot::LiftRaisedToUpperLimitAndArmLowered(double height)
{
	double posArm = PotArm->Get();

	if(LimitLiftHi->Get())
		MotorLift->Set(GetLiftSpeed(-0.75,!LimitLiftLo->Get(),!LimitLiftHi->Get()));
	else
		MotorLift->Set(0.0);

	if(posArm > height)
		MotorArm->Set(GetArmSpeed(-0.5,posArm,7.5,1.5,1.0,0.0));
	else
		MotorArm->Set(0.0);

	if(!LimitLiftHi->Get() && posArm < height) return true;
	else return false;
}


