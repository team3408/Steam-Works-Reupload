#include "WPILib.h"
#include <Timer.h>
#include "SmartDashboard/SmartDashboard.h"
#include <string>
using namespace std;

class Robot: public IterativeRobot
{


private:
	/*
	LiveWindow *lw = LiveWindow::GetInstance();

	std::unique_ptr<frc::Command> autonomousCommand;
	frc::SendableChooser<frc::Command*> chooser;

	SendableChooser *Chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
	*/

	// Conrtollers and Sparks
	Joystick *stick1, *stick2;
	Spark *rearLeft, *frontLeft, *rearRight, *frontRight, *shooter, *climber;
	Encoder *robotDistance;
	double leftWheels;
	double rightWheels;

	// Shooter's ON/OFF
	bool spinWheel;

	// Climber
	bool climberForwardSpin;
	bool climberReverseSpin;

	//Adjustment constant for gyro
	double Kp= 0.1;
	//State of Auto code
	bool done = false;
	//revolution preformed by the gyro
	double revolutions;

	//Enables monitoring of angle
	AnalogGyro *gyro;

	// Timer
	Timer *autotimer;
	double stopTime;

	// Storing values from gyro
	double angleMeasurement;

	// Auto drive class instance
	RobotDrive *myDrive;

	// Camera Code
	// CameraServer *camera;

	// String auto chooser code
	Command *autoChooser;
	string Left = "l";
	string Right = "r";
	string Center = "c";
	string humanInput;


	void RobotInit()
	{

		//Starting Camera Server
		CameraServer::GetInstance()->StartAutomaticCapture();

		//Declaring sparks for drive code
		frontLeft = new Spark(0);
		rearLeft = new Spark(1);

		rearRight = new Spark(2);
		frontRight = new Spark(3);

		// Declaring sparks for shooter
		shooter = new Spark(4);

		// Decalaring sparks for climber
		climber = new Spark(5);


		// Declaring controllers
		stick1 = new Joystick(0); // Port zero
		stick2 = new Joystick(1); // Port one

		// Declaring Variables for Autonomous
		autotimer = new Timer();
		stopTime = 5;

		gyro = new AnalogGyro(0);

		//Sets Up auto drive
		myDrive = new RobotDrive(frontLeft, rearLeft, rearRight, frontRight);

		//Asks the driver which auto mode they would like to run
		cout << "Which auto sir or madam do you want to do?";
		cin >> humanInput;//input option are r,l,c

		///<inversion code>
		//rearLeft->SetInverted(true);
		//frontLeft->SetInverted(true);
	}
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit()
	{
		//sets gyro up
		gyro->Reset();
		gyro->Calibrate();
		//resets angle measurement to 0
		angleMeasurement = gyro->GetAngle();
		//angleMeasurement = angleMeasurement/360;
		SmartDashboard::PutNumber("Gyro Angle", angleMeasurement);

		// Setting/Resetting encoder
		revolutions = 0;

		//ensure auto mode is ready to be run
		done = false;

		Wait(1);
		//begins timing autonomous
		autotimer->Reset();
		autotimer->Start();
	}

	void AutonomousPeriodic()
	{
		//runs left side autonomous
		if(humanInput == "l","L")
		{
			// Encoder gets value
			revolutions = robotDistance -> GetDistance();

			if (revolutions < 4.2)
			{
				//drives at half speed straight forward
				myDrive -> ArcadeDrive(0.0,0.5);
			}

			else if (revolutions >= 4.2 && revolutions <= 8.23)
			{
				//turns 30 degrees right
				myDrive->ArcadeDrive(30,0.0);
				//waits .5 secoonds
				Wait(0.5);
				//drives at 30 degrees at half speed
				myDrive->ArcadeDrive(30,0.5);
				//checks rotations
				revolutions = robotDistance -> GetDistance();
			}

			else
			{
				//stops robot at 8.23 total revolutions
				myDrive->ArcadeDrive(30,0.0);
			}
		}
		//runs the right side of autonomous
		else if(humanInput == "r", "R")
		{
			// Encoder gets value
			revolutions = robotDistance -> GetDistance();

			if (revolutions < 4.2)
			{
				//drives at half speed straight forward
				myDrive -> ArcadeDrive(0.0,0.5);
			}

			else if (revolutions >= 4.2 && revolutions <= 8.23)
			{
				//turns -30 (330) degrees right
				myDrive->ArcadeDrive(-30,0.0);
				//waits .5 secoonds
				Wait(0.5);
				//drives at -30 (330) degrees at half speed
				myDrive->ArcadeDrive(-30,0.5);
				//checks rotations
				revolutions = robotDistance -> GetDistance();
			}

			else
			{
				//stops robot at 8.23 total revolutions
				myDrive->ArcadeDrive(30,0.0);
			}
		}

		else
			{
				// HasPeriodPassed returns true once so we need an extra if.
				if(autotimer->HasPeriodPassed(3.5))
				{
					myDrive->ArcadeDrive(0.0,0.0);
					// Setting variable so we can test again
					done = true;
				}
				else if(done)
				{
					myDrive->ArcadeDrive(0.0,0.0);
				}
				else
				{
					//myDrive->ArcadeDrive(angleMeasurement, 0.65);
					myDrive->ArcadeDrive(0, 0.65);
				}
			}
		angleMeasurement = gyro->GetAngle();
		SmartDashboard::PutNumber("Gyro Angle", angleMeasurement);


/*

	if(autotimer->HasPeriodPassed(3.5))
	{
		myDrive->ArcadeDrive(0.0,0.0);
		done = true;
	}
	else if(done)
	{
		myDrive->ArcadeDrive(0.0,0.0);
	}
	else
	{
		myDrive->ArcadeDrive(angleMeasurement, 0.65);
	}

	//Right-Side Code
	myDrive -> ArcadeDrive(0.0,0.5);
	revolutions = robotDistance -> GetDistance();
	if (revolutions >= 4.2)
	{
		myDrive->ArcadeDrive(-30,0.0);
		robotDistance->Reset();
		Wait(0.5);
		myDrive->ArcadeDrive(-30,0.5);
		revolutions = robotDistance -> GetDistance();
		if (revolutions > 4.03)
		{
			myDrive->ArcadeDrive(-30,0.0);
		}
	}

	//Left-Side Code
		myDrive -> ArcadeDrive(0.0,0.5);
		revolutions = robotDistance -> GetDistance();
		if (revolutions >= 4.2)
		{
			myDrive->ArcadeDrive(30,0.0);
			robotDistance->Reset();
			Wait(500);
			myDrive->ArcadeDrive(30,0.5);
			revolutions = robotDistance -> GetDistance();
			if (revolutions > 4.03)
			{
				myDrive->ArcadeDrive(30,0.0);
			}


		}
*/

	}
	void TeleopInit()
	{

	}
	void TeleopPeriodic()
	{
		//angleMeasurement = gyro->GetAngle();
		//SmartDashboard::PutNumber("Gyro Angle", angleMeasurement);
		//DRIVE CODE
		leftWheels = stick1 -> GetRawAxis(1); //leftwheels doesnt work
		rightWheels = stick1 -> GetRawAxis(5);
		frontLeft -> Set(rightWheels);
		frontRight -> Set(leftWheels);
		rearLeft -> Set(rightWheels);
		rearRight -> Set(leftWheels);

		//SHOOTER CODE
		spinWheel = stick1 -> GetRawButton(1);//Assigned to A button

		if (spinWheel)
		{
			shooter->Set(-0.7);// 70% power backwards(motor was mounted backwards)
		}
		else
		{
			shooter->Set(0);// 0 Power
		}

		//Climber Code
		climberForwardSpin = stick1 -> GetRawButton(2);//Assigned to B button

		climberReverseSpin = stick1 -> GetRawButton(3);//Assigned to X button

		if (climberForwardSpin)
		{					//if B button pressed
			climber->Set(1);//Climber gets 100% power
		}
		else if (climberReverseSpin)
		{					 //if X button pressed
			climber->Set(-1);//Climber gets 100% power backwards
		}
		else
		{
			climber->Set(0);//Otherwise climber gets no power
		}


/*
		leftWheels = stick1 -> GetRawAxis(1);
		rightWheels = stick1 -> GetRawAxis(5);


		myDrive->ArcadeDrive(-leftWheels,rightWheels);*/
	}

	void TestPeriodic()
	{
	}
};

START_ROBOT_CLASS(Robot)
