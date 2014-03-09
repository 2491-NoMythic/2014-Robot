#include <WPILib.h>
#include <math.h>

#define SONAR_TO_INCHES 102.4
#define SONAR_TO_FEET 8.533
#define DRIVE_ENCODER_TO_FEET 0.00434782608695652173913043478261

/*
 * Button Config:
 * DriverStation
 * Analog Input 1: Quicklaunch 1 Time
 * Analog Input 2: Quicklaunch 2 Time
 * Analog Input 3: Autonomous Shoot Time
 * Analog Input 4: Autonomous Drive Time
 * Digital Input 1: Enable Shooting in Autonomous
 * Digital Input 2: Use Sonar in Autonomous
 * Digital Input 3: Enable Automatic Transmission
 * Digital Input 8: Talon Calibration Mode
 * 
 * Left Joystick
 * Y Axis: Drive Left Motors
 * Button 3: Launch
 * Button 2: Launcher Retract
 * Trigger + 4: Quicklaunch 1
 * Trigger + 5: Quicklaunch 2
 * Button 6: Shift High
 * Button 7: Shift Low
 * 
 * Right Joystick
 * Y Axis: Drive Right Motors
 * Button 2: Load In
 * Button 3: Load Out
 * Button 5: Lifter Retract / Pull Lifter Up
 * Button 4: Lifter Out / Put Lifter Down
 * Button 9: Drive encoder distance reset
 * 
 *
 * Port Config:
 * PWM Ports
 * Port 1: Left Motor
 * Port 2: Right Motor
 * Port 5: Launcher Motor 1
 * Port 6: Launcher Motor 2
 *
 * Relay Ports
 * Port 1: Compressor
 * Port 2: Loader Motor
 *
 * Digital Sensor Ports
 * Port 1: Pressure Sensor for Compressor
 *
 * Analog Sensor Ports
 * Port 1: Shooter Potentiometer
 *
 * Solenoid Breakout Ports
 * Port 1: Shift Down
 * Port 2: Shift Up
 * Port 3: Pull Lifter Out / Down
 * Port 4: Push Lifter In / Up
 */


//Define some environment variables to use later.  We should edit these when we know what they actually are.
//In this case, they're potentiometer targets for the shooter arm.

class MainRobot : public SimpleRobot
{
	Joystick *joystickLeft, *joystickRight;
	Talon *motorLeft, *motorRight, *launcherOne, *launcherTwo;
	Solenoid *shiftUp, *shiftDown, *lifterUp, *lifterDown;
	Relay *lifter;
	Compressor *compressor;
	AnalogChannel *shooterPot, *sonar;
	Encoder *driveEncoder;
	Timer *timer;
	DriverStationLCD *driverStationLCD;
	DriverStation *driverStation;

public:
	MainRobot(void)	{
		//Set up joysticks
		joystickLeft = new Joystick(1);
		joystickRight = new Joystick(2);
		
		//Set up motors
		motorLeft = new Talon(1);
		motorRight = new Talon(2);
		launcherOne = new Talon(5);
		launcherTwo = new Talon(6);
	
		//Set up solenoids
		shiftUp = new Solenoid(2);
		shiftDown = new Solenoid(1);
		lifterUp = new Solenoid(4);
		lifterDown = new Solenoid(3);
		
		//Set up the lifter relay
		lifter = new Relay(2);
		
		//Set up the compressor
		compressor = new Compressor(1,1);
		//Start the compressor!
		compressor->Start();
		
		shooterPot = new AnalogChannel(2);
		sonar = new AnalogChannel(1);
		
		driveEncoder = new Encoder(10,11, true, Encoder::k1X);
		driveEncoder->SetDistancePerPulse(DRIVE_ENCODER_TO_FEET);
		driveEncoder->Start();
		
		timer = new Timer();
		timer->Start();
		timer->Reset();
		
		driverStationLCD = DriverStationLCD::GetInstance();
		driverStation = DriverStation::GetInstance();
	}
	void Autonomous(void) {
		driveEncoder->Reset();
		//Stop the compressor
		compressor->Stop();
		//Make sure the lifter is down
		lifterDown->Set(true);
		lifterUp->Set(false);
		//Make sure we're in high gear
		shiftUp->Set(true);
		shiftDown->Set(false);
		//Drive forward for 1.5 seconds...
		motorRight->Set(-0.7);
		motorLeft->Set(0.7);
		//If driverstation switch is on, wait until the sonar is at 9.5 feet.
		if(driverStation->GetDigitalIn(2)){
			while(IsAutonomous() && sonar->GetVoltage() * SONAR_TO_FEET > 9.5) {
				Wait(0.02);
			}
		}
		else { //Otherwise, wait for 1.0 seconds.
			Wait(1.3);
		}
		lifterDown->Set(false);
		shiftUp->Set(false);
		motorRight->Set(0.0);
		motorLeft->Set(0.0);
		//If autonomous shooting is enabled...
		if(driverStation->GetDigitalIn(1)) {
			//Wait a bit...
			Wait(2.0);
			//Shoot!  The shoot time is based on DS analog input 3.
			TimedShot(driverStation->GetAnalogIn(3));
		}
			//Restart the compressor
			compressor->Start();
		
	}
	void OperatorControl(void) {
		driveEncoder->Reset();
		int count = 0;
		bool shifting = false; //Variable for shifting
		int pendingShift = 0; //Stores what if any shifts are pending.  1.0 = shift high, 0.0 = none, -1.0 = shift low
		int currentShift = 0; //Stores what shift we are currently on.  Same as above.
		double shiftTimer = 0.0; //The timeout will set the pending shift to 0.0 if the timer passes this time.
		//double lowFreqTimer = 0.0; //Wasn't working... Stores the next time that the low frequency code runs
		//double shooterTimeout = 0.0; //Backup timeout for shooter in case the sensor malfunctions
		float shooterFarShot = 0.0; //Potentiometer reading at top of a far shot
		float shooterCloseShot = 0.0; //Potentiometer reading at top of a close shot
		float shooterBottom = 0.0; //Potentiometer reading when shooter arm is at the bottom spot
		float transmissionCutoff = driverStation->GetAnalogIn(4);
		bool useAutoShift = driverStation->GetDigitalIn(3);
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
			if(joystickLeft->GetRawButton(3) && !(joystickLeft->GetRawButton(2))) { //If you're pushing 6 and not 7 on the left joystick then launch the launcher!
				printf("Launching!\n"); //Print out the fact that it's happening to the netconsole.
				launcherOne->Set(1.0);
				launcherTwo->Set(1.0);
			}
			else if(joystickLeft->GetRawButton(2) && !(joystickLeft->GetRawButton(3))) {
				printf("Bringing launcher back.\n"); //Print out the fact that it's happening to the netconsole.
				if(driverStation->GetDigitalIn(8)) {
					launcherOne->Set(-1.0);
					launcherTwo->Set(-1.0);
				}
				else {
					launcherOne->Set(-0.2);
					launcherTwo->Set(-0.2);
				}
			}
			else {
				launcherOne->Set(0.0);
				launcherTwo->Set(0.0);
			}
			
			//Quicklaunches
			if (joystickLeft->GetTrigger() && joystickLeft->GetRawButton(4)) { //If you hold down the trigger and push 11...
				//Shoot based on the time set on analog input 1 of the DS
				TimedShot(driverStation->GetAnalogIn(1));
			}
			//Same thing as before, but with different buttons and different IO ports.
			if (joystickLeft->GetTrigger() && joystickLeft->GetRawButton(5)) { //If you hold down the trigger and push 10..
				//Shoot based on the time set on analog input 2 of the DS
				TimedShot(driverStation->GetAnalogIn(2));
			}
			
			//Gearshift control
			if (useAutoShift) {
				int ASCheck = checkForAutoShift(currentShift, transmissionCutoff);
				if (ASCheck == 1) {
					if (currentShift != 1) {
						pendingShift = 1;
					}
				}
				else if (ASCheck == -1) {
					if (currentShift != -1) {
						pendingShift = -1;
					}
				}
			}
			else {
				//If you push the buttons, the code will put it in the pending shift variable and wait until the robot is moving to shift.
				if (joystickLeft->GetRawButton(6) && !(joystickLeft->GetRawButton(7))) {
					pendingShift = 1;
					driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Shift High Pending"); //Print out pending shifts!
				}
				else if (joystickLeft->GetRawButton(7) && !(joystickLeft->GetRawButton(6))) {
					pendingShift = -1;
					driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Shift Low Pending ");
				}
			}
			//if (pendingShift == 1 && (fabs(motorLeft->Get()) > 0.1) && (fabs(motorRight->Get()) > 0.1)) { //Only enable solenoids if we're moving
			if (pendingShift == 1) { //For autoshifting
				driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Shifting High     "); //Print stuff out!  Yeah!
				shiftUp->Set(true);  //Set Solenoids
				shiftDown->Set(false);
				currentShift = 1;
				if (!shifting) {
					shifting = true;  //If this is the start of a shifting operation, set the timer so shifting ends half a second later.
					shiftTimer = timer->Get() + 0.2;
				}
			}
			//Same thing as before except this is for shifting down.
			//else if (pendingShift == -1 && (fabs(motorLeft->Get()) > 0.1) && (fabs(motorRight->Get()) > 0.1)) {
			else if (pendingShift == -1) {
				driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Shifting Low      ");
				shiftUp->Set(false);
				shiftDown->Set(true);
				currentShift = -1;
				if (!shifting) {
					shifting = true;
					shiftTimer = timer->Get() + 0.2;
				}
			}
			else {
				shiftUp->Set(false);
				shiftDown->Set(false);
				if (shifting) { //Reset shifting variable and driverstation LCD
					if (currentShift == 1) {
						driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Shifted High       ");
					}
					else {
						driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Shifted Low       ");
					}
					shifting = false;
				}
			}
			
			//Lifter position control
			if (joystickRight->GetRawButton(5) && !(joystickRight->GetRawButton(4))) {
				printf("Lifter Up!");
				lifterUp->Set(true);
				lifterDown->Set(false);
			}
			else if (joystickRight->GetRawButton(4) && !(joystickRight->GetRawButton(5))) {
				printf("Lifter Down!");
				lifterUp->Set(false);
				lifterDown->Set(true);
			}
			else {
				lifterUp->Set(false);
				lifterDown->Set(false);
			}
			
			//Encoder Reset Button
			if (joystickRight->GetRawButton(9)) {
				driveEncoder->Reset();
			}
			
			//Timer expirations
			if (shifting && timer->HasPeriodPassed(shiftTimer)) { //If the timer passes the time written to the shiftTimer variable, set the pending shift to none.  This will stop the shift the next run of the code
				pendingShift = 0;
			}
			//Stop the compressor if we're launching.
			if (launcherOne->Get() > 0.0) {
				compressor->Stop();
			}
			else {
				compressor->Start();
			}
			//Print the shooter potentiometer voltage to line 3 of the DS LCD
			driverStationLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Shooter: %f", shooterPot->GetVoltage());
			//Print the sonar distance in feet to line 4
			driverStationLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Sonar: %f ft    ", sonar->GetVoltage() * SONAR_TO_FEET);
			//Print the drive speed to line 5
			driverStationLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Speed: %f f/s    ", driveEncoder->GetRate());
			//Print the drive distance to line 6
			driverStationLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Distance: %f ft    ", driveEncoder->GetDistance());
			//These things only run once per 100 runs.  Good for network access.
			if (count % 25 == 0) {
				shooterFarShot = driverStation->GetAnalogIn(1); //Read analog inputs and set variables to them.
				shooterCloseShot = driverStation->GetAnalogIn(2);
				shooterBottom = driverStation->GetAnalogIn(3);
				transmissionCutoff = driverStation->GetAnalogIn(4);
				useAutoShift = driverStation->GetDigitalIn(3);
				driverStationLCD->UpdateLCD(); //Update the DS LCD with new information.
			} 
			//increment the count variable
			++count;
			Wait(0.01);
		}
	}
	//Function to shoot based on a set time
	void TimedShot(float time) {
		//Stop the compressor...
		compressor->Stop();
		//Launch!
		launcherOne->Set(1.0);
		launcherTwo->Set(1.0);
		//Wait for the given time
		Wait(time);
		//Restart the compressor
		compressor->Start();
		//Stop the launcher
		launcherOne->Set(0.0);
		launcherTwo->Set(0.0);
		//Wait a bit to let the launcher stop
		Wait(0.1);
		//Bring back the launcher
		launcherOne->Set(-0.2);
		launcherTwo->Set(-0.2);
		Wait(time*1.5);
		launcherOne->Set(0.0);
		launcherTwo->Set(0.0);
	}
	
	int checkForAutoShift(int currentShift, float transmissionCutoff) {
		//If someone's holding a shift button, do that shift.
		if (joystickLeft->GetRawButton(6)) {
			return 1;
		}
		else if (joystickLeft->GetRawButton(7)) {
			return -1;
		}
		//If the joysticks are opposite each other (neg vs pos) we're turning and shouldn't shift.
		if (joystickRight->GetY() * joystickLeft->GetY() < 0) {
			printf("No shift due to turning\n");
			return 0;
		}
		//Average the joysticks.
		//float joystickAverage = (joystickRight->GetY() + joystickLeft->GetY()) / 2;
		//If the joystick is opposite the the wheel direction, shift low
		if (joystickRight->GetY() * -1 * driveEncoder->GetRate() < 0) {
			printf("Shift low due to direction switch.  Speed: %f\n", joystickRight->GetY());
			return -1;
		}
		//If we're going faster than the transmission cutoff variable (set from DS IO), shift high.
		if (fabs(driveEncoder->GetRate()) > transmissionCutoff) {
			printf("Shift high due to speed\n");
			return 1;
		}
		printf("Shift low due to nothing else\n");
		return -1;
	}
};

START_ROBOT_CLASS(MainRobot);
