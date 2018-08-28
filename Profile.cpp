#include "Profile.h"
#include "PID.h"
#include "Utility.h"
#include <math.h>
#include <vector>
using namespace frc;

Profile::Profile()
{
	Initialize();
}

void Profile::Initialize()
{
	try
	{
		ProfileLoaded = false;
		ProfileContinuous =false;
		ProfileCompleted = false;
		StepNDX = 0;
		ProfileStep = StepNDX;
		StartDistance = 0;
		MoveStartHeading = 0;
		OutputMagnitude = 0;
		Curve = 0;
		TurnPID.Initialize(&ProfileTurnKp,&ProfileTurnKi,&ProfileTurnKd);
		SteerPID.Initialize(&ProfileSteerKp,&ProfileSteerKi,&ProfileSteerKd);
	}
	catch(std::exception& ex)
	{
		std::string err_string = "[Profile_Initialize] ";
		err_string += ex.what();
		printf(err_string.c_str());
	}
}

void Profile::ExecuteProfile(double heading, double distance)
{
	double curDistance = 0;
	static double startError = 0;
	double curError = 0;

	try
	{
		if(Steps.size() > 0)
		{
			curDistance = distance - StartDistance;
			switch((int)Steps[StepNDX].Command) //evaluate command
			{
				//MOVE - drive straight for given distance
				// 1 = Direction
				// 2 = Distance in feet
				case 1:
				{
					if(!Steps[StepNDX].StartFlag) //Start Flag
					{
						Steps[StepNDX].StartFlag = true;
						StartDistance = distance;
						curDistance = distance - StartDistance;
						//reverse the steering gain depending on forward or reverse
						if (Steps[StepNDX].MaxSpeed < 0) ProfileSteerKp = fabs(ProfileSteerKp) * -1;
						else ProfileSteerKp = fabs(ProfileSteerKp);
						if (StepNDX > 0)
							Set_Trapezoid(Steps[StepNDX].TgtDistance,Steps[StepNDX].MinSpeed,Steps[StepNDX].MaxSpeed,Steps[StepNDX-1].MaxSpeed,Steps[StepNDX+1].MaxSpeed); //initialize the move profile
						else
							Set_Trapezoid(Steps[StepNDX].TgtDistance,Steps[StepNDX].MinSpeed,Steps[StepNDX].MaxSpeed,0,Steps[StepNDX+1].MaxSpeed); //initialize the move profile
						MoveStartHeading = heading;
						printf("MOVE %i Start -  Tgt: %5.2f\n",StepNDX,Steps[StepNDX].TgtDistance);
						printf("MOVE %i Start - Dist: %5.2f\n",StepNDX,curDistance);
						printf("MOVE %i StartHeading: %5.2f\n",StepNDX,MoveStartHeading);
					}
					if(!Steps[StepNDX].DoneFlag)
					{
						curError = GetNormalizedError(heading,MoveStartHeading);
						Curve = Clamp(SteerPID.Update(0.0,curError));
						OutputMagnitude = Clamp(Get_Trapezoid(curDistance)); //execute the move profile
						//printf("[ExecuteProfile] dist= %.1f speed=%.2f  curve%.1f\n",curDistance,OutputMagnitude,Curve);
					}
					else
					{
						printf("MOVE %i Done - Dist: %5.2f\n",StepNDX,curDistance);
						printf("MOVE %i EndHeading: %5.2f\n",StepNDX,heading);
						Curve = 0.0;
						OutputMagnitude = 0.0;
						StepNDX++;
					}
					break;
				}
				//TURN - turn to new heading
				// 1 = Heading
				// 2 = Speed
				case 2:
				{
					if(!Steps[StepNDX].StartFlag) //Start Flag
					{
						Steps[StepNDX].StartFlag = true;
						printf("TURN %i Start -   Tgt: %5.2f\n",StepNDX,Steps[StepNDX].TgtHeading);
						printf("TURN %i Start - Angle: %5.2f\n",StepNDX,heading);
						startError = GetNormalizedError(heading,Steps[StepNDX].TgtHeading);
						printf("TURN %i Start - Error: %5.2f\n",StepNDX,startError);
						ProfileTurnKp = fabs(ProfileTurnKp);
					}
					if(!Steps[StepNDX].DoneFlag)
					{
						curError = GetNormalizedError(heading,Steps[StepNDX].TgtHeading);
						Curve = Clamp(TurnPID.Update(0.0,curError));
						//Set curve based on which way we are turning - left = negative
						if (Steps[StepNDX].TurnSpeed < 0) Curve = fabs(Curve) * -1.0;
						else Curve = fabs(Curve);
						//calculate ramp for turning speed
						double speedfactor = curError/startError;
						double ramp = Steps[StepNDX].TurnSpeed * speedfactor;
						if(ramp < 0 && ramp > -0.25) ramp = -0.25;
						if(ramp > 0 && ramp < 0.25)	ramp = 0.25;
						//set speed of outside wheel in turn
						OutputMagnitude = Clamp(fabs(ramp) * -1.0); //always turn forward
						//determine if we have reached target
						if((fabs(curError) < 2.0)) Steps[StepNDX].DoneFlag = true;
					}
					else
					{
						printf("TURN %i Done - Angle: %5.2f\n",StepNDX,heading);
						printf("TURN %i Done - Error: %5.2f\n",StepNDX,curError);
						Curve = 0.0;
						OutputMagnitude = 0.0;
						StepNDX++;
					}
					break;
				}
				//PAUSE - wait for a period of time
				// 1 = pause time in milliseconds
				case 3:
				{
					if(!Steps[StepNDX].StartFlag) //Start Flag
					{
						Steps[StepNDX].StartFlag = true;
						PauseTime = RobotController::GetFPGATime();
						printf("PAUSE - Start\n");
						printf("PAUSE - Duration: %ju\n",uint64_t(Steps[StepNDX].PauseTime));
					}
					ElapsedTime = RobotController::GetFPGATime();
					ElapsedTime = (ElapsedTime - PauseTime) / 1000;
					if(ElapsedTime < uint64_t(Steps[StepNDX].PauseTime))
					{
						Curve = 0.0;
						OutputMagnitude = 0.0;
					}
					else
					{
						Steps[StepNDX].DoneFlag = true; //Done Flag
						printf("PAUSE - Done\n");
						printf("PAUSE - Elapsed %ju\n",ElapsedTime);
						Curve = 0.0;
						OutputMagnitude = 0.0;
						StepNDX++;
					}
					break;
				}
				//CURVE - drive a curve for a given distance
				// 0 = 4 = CURVE Command
				// 1 = Direction
				// 3 = Distance in feet
				// 4 = Curve    (-1 to 1 with -1 to left, +1 to right)
				case 4:
				{
					if(!Steps[StepNDX].StartFlag) //Start Flag
					{
						Steps[StepNDX].StartFlag = true;
						StartDistance = distance;
						curDistance = distance - StartDistance;
						//reverse the steering gain depending on forward or reverse
						if (Steps[StepNDX].MaxSpeed < 0) ProfileSteerKp = fabs(ProfileSteerKp) * -1;
						else ProfileSteerKp = fabs(ProfileSteerKp);
						if (StepNDX > 0)
							Set_Trapezoid(Steps[StepNDX].TgtDistance,Steps[StepNDX].MinSpeed,Steps[StepNDX].MaxSpeed,Steps[StepNDX-1].MaxSpeed,Steps[StepNDX+1].MaxSpeed); //initialize the move profile
						else
							Set_Trapezoid(Steps[StepNDX].TgtDistance,Steps[StepNDX].MinSpeed,Steps[StepNDX].MaxSpeed,0,Steps[StepNDX+1].MaxSpeed); //initialize the move profile
						MoveStartHeading = heading;
						printf("CURVE %i Start -  Tgt: %5.2f\n",StepNDX,Steps[StepNDX].TgtDistance);
						printf("CURVE %i Start - Dist: %5.2f\n",StepNDX,curDistance);
						printf("CURVE %i StartHeading: %5.2f\n",StepNDX,MoveStartHeading);
					}
					if(!Steps[StepNDX].DoneFlag)
					{
						//curError = GetNormalizedError(heading,MoveStartHeading);
						//Curve = Clamp(SteerPID.Update(0.0,curError));
						Curve = Clamp(Steps[StepNDX].Curve); //user controls amount of curve, 0 = straight
						OutputMagnitude = Clamp(Get_Trapezoid(curDistance)); //execute the move profile
					}
					else
					{
						printf("CURVE %i Done - Dist: %5.2f\n",StepNDX,curDistance);
						printf("CURVE %i EndHeading: %5.2f\n",StepNDX,heading);
						Curve = 0.0;
						OutputMagnitude = 0.0;
						StepNDX++;
					}
					break;
				}
				default:
				{
					Curve = 0.0;
					OutputMagnitude = 0.0;
				}
			}

		}
		else
		{	//do nothing
			Curve = 0.0;
			OutputMagnitude = 0.0;
		}
		ProfileStep = StepNDX;
		ProfileCompleted = StepNDX >= Steps.size();
	}
	catch(std::exception& ex)
	{
		std::string err_string = "[ExecuteProfile] ";
		err_string += ex.what();
		printf(err_string.c_str());
	}
}

int Profile::ClearProfile()
{
	try
	{
		ProfileLoaded = false;
		ProfileContinuous = false;
		StepNDX = 0;
		ProfileStep = StepNDX;
		ProfileCompleted = false;
		StartDistance = 0;
		MoveStartHeading = 0;
		OutputMagnitude = 0;
		Curve = 0;
		Steps.clear();
		return 0;
	}
	catch(std::exception& ex)
	{
		std::string err_string = "[ClearProfile] ";
		err_string += ex.what();
		printf(err_string.c_str());
		return 0;
	}
}

int Profile::AddMove(DirectionType Direction, double TgtDistance)
{
	ProfileParams pp;

	try
	{
		pp.Command = 1;
		if (Direction == kProfileForward)
		{
			pp.MinSpeed = fabs(ProfileMinSpeed) * -1;
			pp.MaxSpeed = fabs(ProfileMaxSpeed) * -1;
		}
		else
		{
			pp.MinSpeed = fabs(ProfileMinSpeed);
			pp.MaxSpeed = fabs(ProfileMaxSpeed);
		}
		pp.TgtDistance = TgtDistance;
		pp.StartFlag = false;
		pp.DoneFlag = false;
		Steps.push_back(pp);
		return Steps.size();
	}
	catch(std::exception& ex)
	{
		std::string err_string = "[AddMove] ";
		err_string += ex.what();
		printf(err_string.c_str());
		return 0.0f;
	}
}

int Profile::AddTurn(double TgtHeading, double speed)
{
	ProfileParams pp;

	try
	{
		pp.Command = 2;
		pp.TgtHeading = TgtHeading;
		pp.TurnSpeed = speed;
		pp.StartFlag = false;
		pp.DoneFlag = false;
		Steps.push_back(pp);
		return Steps.size();
	}
	catch(std::exception& ex)
	{
		std::string err_string = "[AddTurn] ";
		err_string += ex.what();
		printf(err_string.c_str());
		return 0.0f;
	}
}

int Profile::AddPause(double mSecs)
{
	ProfileParams pp;

	try
	{
		pp.Command = 3;
		pp.PauseTime = mSecs;
		pp.StartFlag = false;
		pp.DoneFlag = false;
		Steps.push_back(pp);
		return Steps.size();
	}
	catch(std::exception& ex)
	{
		std::string err_string = "[AddPause] ";
		err_string += ex.what();
		printf(err_string.c_str());
		return 0.0f;
	}
}

int Profile::AddCurve(DirectionType Direction, double TgtDistance, double Curve)
{
	ProfileParams pp;

	try
	{
		pp.Command = 4;
		if (Direction == kProfileForward)
		{
			pp.MinSpeed = fabs(ProfileMinSpeed) * -1;
			pp.MaxSpeed = fabs(ProfileMaxSpeed) * -1;
		}
		else
		{
			pp.MinSpeed = fabs(ProfileMinSpeed);
			pp.MaxSpeed = fabs(ProfileMaxSpeed);
		}
		pp.TgtDistance = TgtDistance;
		pp.Curve = Curve;
		pp.StartFlag = false;
		pp.DoneFlag = false;
		Steps.push_back(pp);
		return Steps.size();
	}
	catch(std::exception& ex)
	{
		std::string err_string = "[AddCurve] ";
		err_string += ex.what();
		printf(err_string.c_str());
		return 0.0f;
	}
}

//Parameters are in 0-360 degrees
double Profile::GetNormalizedError(double heading, double newHeading)
{
	double rawError;
	//normalize incoming headings and subtract to get error
	rawError = GetNormalizedHeading(newHeading) - GetNormalizedHeading(heading);
	//normalize again to get shortest distance to steer
	//negative number steers left and positive steers right
	return GetNormalizedHeading(rawError);
}

double Profile::GetNormalizedHeading(double heading)
{
	if(heading > 180) return heading -= 360;
	else if(heading < -180) return heading += 360;
	else return heading;
}

double Profile::Clamp(double steerRate)
{
	double steerClamp = steerRate;
	if(steerClamp < -1.0) steerClamp = -1.0;
	if(steerClamp > 1.0) steerClamp = 1.0;
	return steerClamp;
}

void Profile::Set_Trapezoid(double tgtValue, double minSpeed, double maxSpeed, double lastSpeed, double nextSpeed)
{
	try
	{
		MoveTarget = abs(tgtValue);
		MoveSteps = (MoveTarget/0.25) * 12;  // 1/4" slices with given distance in feet
		MoveMinSpeed = minSpeed;
		MoveMaxSpeed = maxSpeed;
		MoveLastSpeed = lastSpeed;
		MoveNextSpeed = nextSpeed;
		MoveFirstCorner = MoveSteps/8;  //ramp = 1/8 of total distance
		MoveSecondCorner = MoveFirstCorner * 7;
		MoveSlice = MoveTarget/MoveSteps;
		MoveAccel = (MoveMaxSpeed - MoveMinSpeed) / (MoveFirstCorner * MoveSlice);
		MoveStartTime = RobotController::GetFPGATime();
	}
	catch(std::exception& ex)
	{
		std::string err_string = "[Set_Trapezoid] ";
		err_string += ex.what();
		printf(err_string.c_str());
	}
}

//This is a trapezoidal motion profile based on discrete distance steps
double Profile::Get_Trapezoid(double curDist)
{
	double outSpeed = 0.0f;
	double curStep = 0;

	try
	{
		curStep = abs(round(abs(curDist)/MoveSlice));
		if(curStep < MoveSteps)
		{
			if(curStep <= MoveFirstCorner)
			{
				outSpeed = (MoveAccel * abs(curDist)) + MoveMinSpeed;
				if(ProfileContinuous && StepNDX > 0) //check for first step
				{
					if((int)Steps[StepNDX-1].Command != 3) //check for pause as last step
						if (outSpeed <  MoveLastSpeed) outSpeed = MoveLastSpeed;
				}
				//make sure MoveMinSpeed is not too low for the robot to move
				if(MoveDistCount < 5) MoveDistCount++;
				else
				{
					MoveDistCount = 0;
					//if we have not moved then bump speed by 10%
					if(fabs(curDist) < MoveSlice)
					{
						if(MoveMinSpeed > 0)
						{
							MoveMinSpeed += (MoveMaxSpeed - MoveMinSpeed)/10;
							if(MoveMinSpeed > MoveMaxSpeed) MoveMinSpeed = MoveMaxSpeed;
						}
						else
						{
							MoveMinSpeed -= (MoveMaxSpeed - MoveMinSpeed)/10;
							if(MoveMinSpeed < MoveMaxSpeed) MoveMinSpeed = MoveMaxSpeed;
						}
					}

				}
			}
			if(curStep > MoveFirstCorner && curStep < MoveSecondCorner) outSpeed = MoveMaxSpeed;
			if(curStep >= MoveSecondCorner)
			{
				outSpeed = (MoveAccel * (MoveTarget - abs(curDist)))+ MoveMinSpeed;
				if(ProfileContinuous && StepNDX < Steps.size() - 1)
				{
					if((int)Steps[StepNDX+1].Command != 3) //check for pause as next step
					{
						if(MoveMaxSpeed > MoveNextSpeed)
						{
							if (outSpeed <  MoveNextSpeed) outSpeed = MoveNextSpeed;
						}
						else
						{
							if (outSpeed >  MoveNextSpeed) outSpeed = MoveNextSpeed;
						}
					}
				}
			}
			//printf("step= %f, dist= %5.2f, outspeed= %5.2f\n",curStep,curDist,outSpeed);
			return outSpeed;
		}
		else
		{
			Steps[StepNDX].DoneFlag = true; //Done Flag
			ElapsedTime = (RobotController::GetFPGATime() - MoveStartTime) / 1000;
			printf("Motion Time = %ju ms\n",ElapsedTime);
			return 0.0f;
		}
	}
	catch(std::exception& ex)
	{
		std::string err_string = "[Get_Trapezoid] ";
		err_string += ex.what();
		printf(err_string.c_str());
		return 0.0f;
	}
}







