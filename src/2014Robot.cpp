#include <WPILib.h>
#include <math.h>

/*
 * Button Config:
 * DriverStation
 * Analog Input 1: Quicklaunch 1 Time
 * Analog Input 2: Quicklaunch 2 Time
 * 
 * Left Joystick
 * Y Axis: Drive Left Motors
 * Button 6: Launch
 * Button 7: Launcher Retract
 * Trigger + 12: Quicklaunch 2
 * Trigger + 12: Quicklaunch 1
 * 
 * Right Joystick
 * Y Axis: Drive Right Motors
 * Button 2: Load In
 * Button 3: Load Out
 * Button 6: Lifter Retract / Pull Lifter Up
 * Button 7: Lifter Out / Put Lifter Down
 * Button 10: Shift Low
 * Button 11: Shift High
 */


//Define some environment variables to use later.  We should edit these when we know what they actually are.
//In this case, they're potentiometer targets for the shooter arm.

class TestRobot : public SimpleRobot
{
	Joystick *joystickLeft, *joystickRight;
	Talon *motorLeft, *motorRight, *launcherOne, *launcherTwo;
	Solenoid *shiftUp, *shiftDown, *lifterUp, *lifterDown;
	Relay *lifter;
	Compressor *compressor;
	AnalogChannel *shooterPot;
	Timer *timer;
	DriverStationLCD *driverStationLCD;
	DriverStation *driverStation;

public:
	TestRobot(void)	{
		//Set up joysticks
		joystickLeft = new Joystick(1);
		joystickRight = new Joystick(2);
		
		//Set up motors
		motorLeft = new Talon(2);
		motorRight = new Talon(1);
		launcherOne = new Talon(5);
		launcherTwo = new Talon(6);
		
		//Set up solenoids
		shiftUp = new Solenoid(2);
		shiftDown = new Solenoid(1);
		lifterUp = new Solenoid(3);
		lifterDown = new Solenoid(4);
		
		//Set up the lifter relay
		lifter = new Relay(2);
		
		//Set up the compressor
		compressor = new Compressor(1,1);
		//Start the compressor!
		compressor->Start();
		
		shooterPot = new AnalogChannel(1);
		
		timer = new Timer();
		timer->Start();
		timer->Reset();
		
		driverStationLCD = DriverStationLCD::GetInstance();
		driverStation = DriverStation::GetInstance();
	}
	void Autonomous(void) {
		motorRight->Set(0.7);
		motorLeft->Set(0.7);
		Wait(1.5);
		motorRight->Set(0.0);
		motorLeft->Set(0.0);
	}
	void OperatorControl(void) {
		bool shifting = false;
		int pendingShift = 0;
		double shiftTimer = 0.0;
		double lowFreqTimer = 0.0;
		double shooterTimeout = 0.0;
		float shooterFarShot = 0.0;
		float shooterCloseShot = 0.0;
		float shooterBottom = 0.0;
		while (IsOperatorControl()) {  //We only want this to run while in teleop mode!
			// Make the robot drive!
			if(fabs(joystickLeft->GetY()) > 0.05) {
				motorLeft->Set(joystickLeft->GetY() * fabs(joystickLeft->GetY()));
			}
			else {
				motorLeft->Set(0.0);
			}

			if(fabs(joystickRight->GetY()) > 0.05) {
				motorRight->Set(-1.0 * joystickRight->GetY() * fabs(joystickRight->GetY()));
			}
			else{
				motorRight->Set(0.0);
			}
			
			//Lifter control
			if(joystickRight->GetRawButton(2) && !(joystickRight->GetRawButton(3))) {
				lifter->Set(Relay::kForward);
				driverStationLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Loading In! ");
			}
			else if(joystickRight->GetRawButton(3) && !(joystickRight->GetRawButton(2))) {
				lifter->Set(Relay::kReverse);
				driverStationLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Loading Out!");
			}
			else {
				driverStationLCD->Printf(DriverStationLCD::kUser_Line1, 1, "            ");
				lifter->Set(Relay::kOff);
			}
			
			//Launcher control
			if(joystickLeft->GetRawButton(6) && !(joystickLeft->GetRawButton(7))) {
				printf("Launching!\n");
				launcherOne->Set(1.0);
				launcherTwo->Set(1.0);
			}
			else if(joystickLeft->GetRawButton(7) && !(joystickLeft->GetRawButton(6))) {
				printf("Bringing launcher back.\n");
				launcherOne->Set(-0.3);
				launcherTwo->Set(-0.3);
			}
			else {
				launcherOne->Set(0.0);
				launcherTwo->Set(0.0);
			}
			
			//Quicklaunches
			if (joystickLeft->GetTrigger() && joystickLeft->GetRawButton(11)) {
				launcherOne->Set(1.0);
				launcherTwo->Set(1.0);
				Wait(driverStation->GetAnalogIn(1));
				launcherOne->Set(0.0);
				launcherTwo->Set(0.0);
				Wait(0.1);
				launcherOne->Set(-0.3);
				launcherTwo->Set(-0.3);
				Wait(driverStation->GetAnalogIn(1)* 2);
				launcherOne->Set(0.0);
				launcherTwo->Set(0.0);
			}
			
			if (joystickLeft->GetTrigger() && joystickLeft->GetRawButton(10)) {
				launcherOne->Set(1.0);
				launcherTwo->Set(1.0);
				Wait(driverStation->GetAnalogIn(2));
				launcherOne->Set(0.0);
				launcherTwo->Set(0.0);
				Wait(0.1);
				launcherOne->Set(-0.3);
				launcherTwo->Set(-0.3);
				Wait(driverStation->GetAnalogIn(2)*2);
				launcherOne->Set(0.0);
				launcherTwo->Set(0.0);
			}
			
			//Gearshift control
			if (joystickRight->GetRawButton(11) && !(joystickRight->GetRawButton(10))) {
				pendingShift = 1;
				driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Shift High Pending");
			}
			else if (joystickRight->GetRawButton(10) && !(joystickRight->GetRawButton(11))) {
				pendingShift = -1;
				driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Shift Low Pending ");
			}
			
			if (pendingShift == 1 && (fabs(motorLeft->Get()) > 0.1) && (fabs(motorRight->Get()) > 0.1)) {
				driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Shifting High     ");
				shiftUp->Set(true);
				shiftDown->Set(false);
				if (!shifting) {
					shifting = true;
					shiftTimer = timer->Get() + 0.5;
				}
			}
			else if (pendingShift == -1 && (fabs(motorLeft->Get()) > 0.1) && (fabs(motorRight->Get()) > 0.1)) {
				driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Shifting Low      ");
				shiftUp->Set(false);
				shiftDown->Set(true);
				if (!shifting) {
					shifting = true;
					shiftTimer = timer->Get() + 0.5;
				}
			}
			else {
				shiftUp->Set(false);
				shiftDown->Set(false);
				if (shifting) {
					driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1, "                  ");
					shifting = false;
				}
			}
			
			//Lifter position control
			if (joystickRight->GetRawButton(6) && !(joystickRight->GetRawButton(7))) {
				printf("Lifter Up!");
				lifterUp->Set(true);
				lifterDown->Set(false);
			}
			else if (joystickRight->GetRawButton(7) && !(joystickRight->GetRawButton(6))) {
				printf("Lifter Down!");
				lifterUp->Set(false);
				lifterDown->Set(true);
			}
			else {
				lifterUp->Set(false);
				lifterDown->Set(false);
			}
			
			//Timer expirations
			if (shifting && timer->HasPeriodPassed(shiftTimer)) {
				pendingShift = 0;
			}
			driverStationLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Shooter: %f", shooterPot->GetVoltage());
			
			//These things only run 10 times a second.  Good for network access.
			if (timer->HasPeriodPassed(lowFreqTimer)) {
				lowFreqTimer = lowFreqTimer + 0.1;
				shooterFarShot = driverStation->GetAnalogIn(1);
				shooterCloseShot = driverStation->GetAnalogIn(2);
				shooterBottom = driverStation->GetAnalogIn(3);
				driverStationLCD->UpdateLCD();
			} 
		}
	}
};
START_ROBOT_CLASS(TestRobot);
