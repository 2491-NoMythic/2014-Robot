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
 * Button 6: Lifter Retract / Pull Lifter Up
 * Button 7: Lifter Out / Put Lifter Down
 * 
 * Right Joystick
 * Y Axis: Drive Right Motors
 * Trigger: Shifting (Off = Low, On = High)
 * Button 2: Load In
 * Button 3: Load Out
 * Button 9: Drive encoder distance reset
 * 
 *
 * Port Config:
 * PWM Ports
 * Port 1: Left Motor
 * Port 2: Right Motor
 * Port 5: Launcher Motor 1
 * Port 6: Launcher Motor 2
 * Port 7: Loader Motor 1
 * Port 8: Loader Motor 2
 *
 * Relay Ports
 * Port 1: Compressor
 * Port 2: Loader Motor (Now two talons)
 *
 * Digital Sensor Ports
 * Port 1: Pressure Sensor for Compressor
 * Port 10: Right Encoder A
 * Port 11: Right Encoder B
 * Port 12: Left Encoder A
 * Port 13: Left Encoder B
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
	Talon *motorLeft, *motorRight, *launcherOne, *launcherTwo, *loaderOne, *loaderTwo;
	Solenoid *shiftUp, *shiftDown, *lifterUp, *lifterDown;
	Compressor *compressor;
	AnalogChannel *shooterPot, *sonar;
	Encoder *encoderRight, *encoderLeft;
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
		
		//Set up the loader motors
		loaderOne = new Talon(7);
		loaderTwo = new Talon(8);
		
		//Set up the compressor
		compressor = new Compressor(1,1);
		//Start the compressor!
		compressor->Start();
		
		shooterPot = new AnalogChannel(2);
		sonar = new AnalogChannel(1);
		
		encoderRight = new Encoder(10,11, true, Encoder::k1X);
		encoderRight->SetDistancePerPulse(DRIVE_ENCODER_TO_FEET);
		encoderRight->Start();
		encoderLeft = new Encoder(12,13, false, Encoder::k1X);
		encoderLeft->SetDistancePerPulse(DRIVE_ENCODER_TO_FEET);
		encoderLeft->Start();
		
		timer = new Timer();
		timer->Start();
		timer->Reset();
		
		driverStationLCD = DriverStationLCD::GetInstance();
		driverStation = DriverStation::GetInstance();
	}
	void Autonomous(void) {
		encoderRight->Reset();
		encoderLeft->Reset();
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
		//If driverstation switch is off, wait until the sonar is at 9.5 feet.
		if(!driverStation->GetDigitalIn(2)){
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
			float autoShootTime = driverStation->GetAnalogIn(3);
			if (autoShootTime == 0) {
				autoShootTime = 3.0;
			}
			TimedShot(autoShootTime);
		}
			//Restart the compressor
			compressor->Start();
		
	}
	void OperatorControl(void) {
		encoderRight->Reset();
		encoderLeft->Reset();
		int count = 0;
		float loaderSpeed = joystickRight->GetZ()/2+0.5;
		bool shifting = false; //Variable for shifting
		int pendingShift = 0; //Stores what if any shifts are pending.  1.0 = shift high, 0.0 = none, -1.0 = shift low
		int currentShift = 0; //Stores what shift we are currently on.  Same as above.
		double shiftTimer = 0.0; //The timeout will set the pending shift to 0.0 if the timer passes this time.
		//double lowFreqTimer = 0.0; //Wasn't working... Stores the next time that the low frequency code runs
		//double shooterTimeout = 0.0; //Backup timeout for shooter in case the sensor malfunctions
		float quickShotA = driverStation->GetAnalogIn(1);
		float quickShotB = driverStation->GetAnalogIn(2);
		float transmissionCutoff = driverStation->GetAnalogIn(4) + 1.0;
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
				loaderOne->Set(loaderSpeed);
				loaderTwo->Set(-1.0 * loaderSpeed);
				driverStationLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Loading In! "); //Print out the fact that it's happening (maybe I should send this to the netconsole...)
			}
			else if(joystickRight->GetRawButton(3) && !(joystickRight->GetRawButton(2))) { //If you're pushing 3 and not 2, enable it backwards.
				loaderOne->Set(-1.0 * loaderSpeed);
				loaderTwo->Set(loaderSpeed);
				driverStationLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Loading Out!"); //Print out the fact that it's happening
			}
			else { //If neither of the previous are true, then disable the lifter motor.
				driverStationLCD->Printf(DriverStationLCD::kUser_Line1, 1, "            ");
				loaderOne->Set(0.0);
				loaderTwo->Set(0.0);
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
				TimedShot(quickShotA);
			}
			//Same thing as before, but with different buttons and different IO ports.
			if (joystickLeft->GetTrigger() && joystickLeft->GetRawButton(5)) { //If you hold down the trigger and push 10..
				//Shoot based on the time set on analog input 2 of the DS
				TimedShot(quickShotB);
			}
			
			//Gearshift control
			if (useAutoShift) {
				int ASCheck = checkForAutoShift(transmissionCutoff);
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
				if (joystickRight->GetTrigger()) {
					pendingShift = 1;
					driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Shift High Pending"); //Print out pending shifts!
				}
				else {
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
			if (joystickLeft->GetRawButton(6) && !(joystickLeft->GetRawButton(7))) {
				printf("Lifter Up!");
				lifterUp->Set(true);
				lifterDown->Set(false);
			}
			else if (joystickLeft->GetRawButton(7) && !(joystickLeft->GetRawButton(6))) {
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
				encoderRight->Reset();
				encoderLeft->Reset();
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
			//Update the loader speed variable
			loaderSpeed = joystickRight->GetZ()/2+0.5;
			//Print the shooter potentiometer voltage to line 3 of the DS LCD
			driverStationLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Shooter: %f", shooterPot->GetVoltage());
			//Print the sonar distance in feet to line 4
			driverStationLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Sonar: %f ft    ", sonar->GetVoltage() * SONAR_TO_FEET);
			//Print the drive speed to line 5
			driverStationLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Right Speed: %f f/s    ", encoderRight->GetRate());
			//Print the drive distance to line 6
			driverStationLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Left Speed:  %f ft/s   ", encoderLeft->GetRate());
			//Print the loader speed to line 1
			driverStationLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Loader Speed: %f\%", loaderSpeed);
			//These things only run once per 100 runs.  Good for network access.
			if (count % 25 == 0) {
				quickShotA = driverStation->GetAnalogIn(1);
				if (quickShotA == 0) {
					quickShotA = 3.0;
				}
				quickShotB = driverStation->GetAnalogIn(2);
				if (quickShotB == 0) {
					quickShotB = 2.4;
				}
				transmissionCutoff = driverStation->GetAnalogIn(4);
				useAutoShift = !driverStation->GetDigitalIn(3);
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
	
	int checkForAutoShift(float transmissionCutoff) {
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
		float encoderRightSpeed = encoderRight->GetRate();
		float encoderLeftSpeed = encoderLeft->GetRate();
		//If the wheels are going opposite each other, still don't shift
		if (encoderRightSpeed * encoderLeftSpeed < 0) {
			printf("No shift due to wheel turning\n");
			return 0;
		}
		//If the joystick is opposite the the wheel direction, shift low
		if (joystickRight->GetY() * 1 * encoderRightSpeed < 0) {
			printf("Shift low due to direction switch.  Speed: %f\n", joystickRight->GetY());
			return -1;
		}
		float encoderAverageRate = (fabs(encoderRightSpeed) + fabs(encoderLeftSpeed))/2;
		//If we're going faster than the transmission cutoff variable (set from DS IO), shift high.
		if (encoderAverageRate > transmissionCutoff + 0.2) {
			printf("Shift high due to speed\n");
			return 1;
		}
		else if (encoderAverageRate < transmissionCutoff - 0.2){
			printf("Shift low due to low speed\n");
			return -1;
		}
		printf("No shift due to dead zone");
		return 0;
	}
};

START_ROBOT_CLASS(MainRobot);
