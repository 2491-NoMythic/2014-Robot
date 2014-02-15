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
		//Drive forward for 1.5 seconds...
		motorRight->Set(-0.7);
		motorLeft->Set(0.7);
		Wait(1.5);
		motorRight->Set(0.0);
		motorLeft->Set(0.0);
		//Wait a bit...
		Wait(1.0);
		//Shoot!  The shoot time is based on DS analog input 3.
		launcherOne->Set(1.0);
		launcherTwo->Set(1.0);
		Wait(driverStation->GetAnalogIn(3));
		launcherOne->Set(0.0);
		launcherTwo->Set(0.0);
		Wait(0.1);
		launcherOne->Set(-0.3);
		launcherTwo->Set(-0.3);
		Wait(driverStation->GetAnalogIn(3)*2);
		launcherOne->Set(0.0);
		launcherTwo->Set(0.0);
		
	}
	void OperatorControl(void) {
		bool shifting = false; //Variable for shifting
		int pendingShift = 0; //Stores what if any shifts are pending.  1.0 = shift high, 0.0 = none, -1.0 = shift low
		double shiftTimer = 0.0; //The timeout will set the pending shift to 0.0 if the timer passes this time.
		double lowFreqTimer = 0.0; //Stores the next time that the low frequency code runs
		double shooterTimeout = 0.0; //Backup timeout for shooter in case the sensor malfunctions
		float shooterFarShot = 0.0; //Potentiometer reading at top of a far shot
		float shooterCloseShot = 0.0; //Potentiometer reading at top of a close shot
		float shooterBottom = 0.0; //Potentiometer reading when shooter arm is at the bottom spot
		while (IsOperatorControl()) {  //We only want this to run while in teleop mode!
			// Make the robot drive!
			if(fabs(joystickLeft->GetY()) > 0.05) { //Only run motors if the joystick is past 5% from middle
				motorLeft->Set(joystickLeft->GetY() * fabs(joystickLeft->GetY())); //Set the left motor to the square of the left joystick.
			}
			else {
				motorLeft->Set(0.0);
			}
			//Same as above
			if(fabs(joystickRight->GetY()) > 0.05) {
				motorRight->Set(-1.0 * joystickRight->GetY() * fabs(joystickRight->GetY()));
			}
			else{
				motorRight->Set(0.0);
			}
			
			//Lifter control
			if(joystickRight->GetRawButton(2) && !(joystickRight->GetRawButton(3))) { //If you're pushing button 2 and not 3 on the right joystick then enable the lifter forward!
				lifter->Set(Relay::kForward);
				driverStationLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Loading In! "); //Print out the fact that it's happening (maybe I should send this to the netconsole...)
			}
			else if(joystickRight->GetRawButton(3) && !(joystickRight->GetRawButton(2))) { //If you're pushing 3 and not 2, enable it backwards.
				lifter->Set(Relay::kReverse);
				driverStationLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Loading Out!"); //Print out the fact that it's happening
			}
			else { //If neither of the previous are true, then disable the lifter motor.
				driverStationLCD->Printf(DriverStationLCD::kUser_Line1, 1, "            ");
				lifter->Set(Relay::kOff);
			}
			
			//Launcher control
			if(joystickLeft->GetRawButton(6) && !(joystickLeft->GetRawButton(7))) { //If you're pushing 6 and not 7 on the left joystick then launch the launcher!
				printf("Launching!\n"); //Print out the fact that it's happening to the netconsole.
				launcherOne->Set(1.0);
				launcherTwo->Set(1.0);
			}
			else if(joystickLeft->GetRawButton(7) && !(joystickLeft->GetRawButton(6))) {
				printf("Bringing launcher back.\n"); //Print out the fact that it's happening to the netconsole.
				launcherOne->Set(-0.3);
				launcherTwo->Set(-0.3);
			}
			else {
				launcherOne->Set(0.0);
				launcherTwo->Set(0.0);
			}
			
			//Quicklaunches
			if (joystickLeft->GetTrigger() && joystickLeft->GetRawButton(11)) { //Only fire if both trigger and button are held
				//Enable launcher motor for time specified in DriverStation analog IO
				launcherOne->Set(1.0);
				launcherTwo->Set(1.0);
				Wait(driverStation->GetAnalogIn(1));
				//Disable motors for 0.1 seconds to slow down
				launcherOne->Set(0.0);
				launcherTwo->Set(0.0);
				Wait(0.1);
				//Set motors backwards for two times the time specified in DS analog IO
				launcherOne->Set(-0.3);
				launcherTwo->Set(-0.3);
				Wait(driverStation->GetAnalogIn(1)* 2);
				//Disable motors
				launcherOne->Set(0.0);
				launcherTwo->Set(0.0);
			}
			//Same thing as before, but with different buttons and different IO ports.
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
			//If you push the buttons, the code will put it in the pending shift variable and wait until the robot is moving to shift.
			if (joystickRight->GetRawButton(11) && !(joystickRight->GetRawButton(10))) {
				pendingShift = 1;
				driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Shift High Pending"); //Print out pending shifts!
			}
			else if (joystickRight->GetRawButton(10) && !(joystickRight->GetRawButton(11))) {
				pendingShift = -1;
				driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Shift Low Pending ");
			}
			
			if (pendingShift == 1 && (fabs(motorLeft->Get()) > 0.1) && (fabs(motorRight->Get()) > 0.1)) { //Only enable solenoids if we're moving
				driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Shifting High     "); //Print stuff out!  Yeah!
				shiftUp->Set(true);  //Set Solenoids
				shiftDown->Set(false);
				if (!shifting) {
					shifting = true;  //If this is the start of a shifting operation, set the timer so shifting ends half a second later.
					shiftTimer = timer->Get() + 0.5;
				}
			}
			//Same thing as before except this is for shifting down.
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
				if (shifting) { //Reset shifting variable and driverstation LCD
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
			if (shifting && timer->HasPeriodPassed(shiftTimer)) { //If the timer passes the time written to the shiftTimer variable, set the pending shift to none.  This will stop the shift the next run of the code
				pendingShift = 0;
			}
			driverStationLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Shooter: %f", shooterPot->GetVoltage()); //Print the shooter potentiometer voltage to line 3 of the DS LCD
			
			//These things only run 10 times a second.  Good for network access.
			if (timer->HasPeriodPassed(lowFreqTimer)) {
				lowFreqTimer = lowFreqTimer + 0.1; //Increment the timer for this stuff.
				shooterFarShot = driverStation->GetAnalogIn(1); //Read analog inputs and set variables to them.
				shooterCloseShot = driverStation->GetAnalogIn(2);
				shooterBottom = driverStation->GetAnalogIn(3);
				driverStationLCD->UpdateLCD(); //Update the DS LCD with new information.
			} 
		}
	}
};
START_ROBOT_CLASS(TestRobot);
