#include <WPILib.h>
#include <math.h>

//Define some environment variables to use later.  We should edit these when we know what they actually are.
//In this case, they're potentiometer targets for the shooter arm.
#define SHOOTER_FARSHOT 1
#define SHOOTER_CLOSESHOT 0.6
#define SHOOTER_BOTTOMOUT 0

class TestRobot : public SimpleRobot
{
	Joystick *joystickLeft, *joystickRight;
	Talon *motorLeft, *motorRight, *launcher;
	Solenoid *shiftUp, *shiftDown, *lifterUp, *lifterDown;
	Relay *lifter, *compressor;
	DigitalInput compressorDetector;
	//Compressor *compressor;
	DriverStationLCD *driverStationLCD;
	DriverStation *driverStation;

public:
	TestRobot(void) : compressorDetector(1)	{
		//Set up joysticks
		joystickLeft = new Joystick(1);
		joystickRight = new Joystick(2);
		
		//Set up motors
		motorLeft = new Talon(2);
		motorRight = new Talon(1);
		launcher = new Talon(5);
		
		//Set up solenoids
		shiftUp = new Solenoid(1);
		shiftDown = new Solenoid(2);
		lifterUp = new Solenoid(3);
		lifterDown = new Solenoid(4);
		
		//Set up the lifter relay
		lifter = new Relay(2);
		
		//Set up the compressor
		//compressor = new Compressor(1,1);
		//Start the compressor!
		//compressor->Start();
		compressor = new Relay(1,Relay::kForwardOnly);
		
		//compressorDetector = new DigitalInput(1);
		
		driverStationLCD = DriverStationLCD::GetInstance();
		driverStation = DriverStation::GetInstance();
	}
	void Autonomous(void) {
		// No autonomous code, this is a test robot!
	}
	void OperatorControl(void) {
		//compressor->Start();
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
			
			//Testing!
			if (joystickLeft->GetRawButton(8)) {
				compressor->Set(Relay::kOn);
			}
			else {
				compressor->Set(Relay::kOff);
			}
			
			if(compressorDetector.Get()) {
				driverStationLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Compressor ON ");
			}
			else {
				driverStationLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Compressor OFF");
			}
			
			//Lifter Control
			if(joystickRight->GetRawButton(3) && !(joystickRight->GetRawButton(2))) {
				lifter->Set(Relay::kForward);
				driverStationLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Loading In! ");
			}
			else if(joystickRight->GetRawButton(2) && !(joystickRight->GetRawButton(3))) {
				lifter->Set(Relay::kReverse);
				driverStationLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Loading Out!");
			}
			else {
				driverStationLCD->Printf(DriverStationLCD::kUser_Line1, 1, "            ");
				lifter->Set(Relay::kOff);
			}
			
			//Launcher control
			if(joystickLeft->GetRawButton(6) && !(joystickLeft->GetRawButton(7))) {
				printf("Launching!");
				launcher->Set(1.0);
			}
			else if(joystickLeft->GetRawButton(7) && !(joystickLeft->GetRawButton(6))) {
				printf("Bringing launcher back.");
				launcher->Set(-0.3);
			}
			else {
				launcher->Set(0.0);
			}
			
			//Quicklaunches
			if (joystickLeft->GetTrigger() && joystickLeft->GetRawButton(11)) {
				launcher->Set(1.0);
				Wait(driverStation->GetAnalogIn(1));
				launcher->Set(0.0);
				Wait(0.1);
				launcher->Set(-0.3);
				Wait(driverStation->GetAnalogIn(1)*1.5);
			}
			
			if (joystickLeft->GetTrigger() && joystickLeft->GetRawButton(10)) {
				launcher->Set(1.0);
				Wait(driverStation->GetAnalogIn(2));
				launcher->Set(0.0);
				Wait(0.1);
				launcher->Set(-0.3);
				Wait(driverStation->GetAnalogIn(2)*1.5);
			}
			
			//Gearshift conrol
			if (joystickRight->GetRawButton(11) && !(joystickRight->GetRawButton(10))) {
				shiftUp->Set(true);
				shiftDown->Set(false);
			}
			else if (joystickRight->GetRawButton(10) && !(joystickRight->GetRawButton(11))) {
				shiftUp->Set(false);
				shiftDown->Set(true);
			}
			else {
				shiftUp->Set(false);
				shiftDown->Set(false);
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
			driverStationLCD->UpdateLCD();
		}
	}
};
START_ROBOT_CLASS(TestRobot);
