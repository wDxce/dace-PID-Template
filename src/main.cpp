#include "main.h"

dace::Drive chassis(

	{8,-9,10},	//Left side of chassis motors 8, -9, 10
	{-1,2,3},	//Right side of chassis motors -1, 2, 3

	11,			//imu port
	3.25,		//wheel diameter in inches
	450,		//Wheel RPM = cartridge * (motor gear / wheel gear)
	blue 		//Motor cartridge Red(100 RPM) -- Green(200 RPM) -- Blue(600 RPM)

);


void initialize() {
	chassis.begin();
 	// Set PID values for all movement types
    dace::PID_Values(
        0.60, 0.00, 3.50,   // drive:  P, I, D
        2.00, 0.00, 0.25,  // turn:   P, I, D
        1.50, 0.00, 0.20,  // swing:  P, I, D
        1.20, 0.00, 0.15   // curve:  P, I, D
    );

	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD); //set brake time
}


void disabled() {}

void competition_initialize() {}


void autonomous() {
	test();
}


void opcontrol() {

	while (true) {
		//Pick a driving style

		/*
		the number in the () is the deadzone 10 is an ideal number
		*/

		//Uses left analog Y for fwd/rev and right analog X for lateral motion
		dace::arcade_drive(10);

		//Uses left analog Y to control the left side of chassis
		//Uses right Analog Y to control right side of chassis
		//Push both up/down at same time to go fwd/rev
		//dace::tank_drive(10);

		//Uses one sitck for fwd/rev (Y) and lateral motion (X)
		//dace::single_stick_drive(dace::Stick::Right, 10);
		//dace::single_stick_drive(dace::Stick::Left, 10);

		//Examples of subsystems(check subsystems.hpp and .cpp to add personal additions)     
		                       
		// Run for 10 ms then update
		pros::delay(10);
	}
}