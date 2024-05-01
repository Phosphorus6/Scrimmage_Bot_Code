#include "../include/main.h"
#include <map>

#define left_motor_1 1
//#define left_motor_2 2
//#define left_motor_3 3
#define right_motor_1 2
//#define right_motor_2 5
//#define right_motor_3 6

#define climb_motor_port 20
#define climb_motor_port 19

#define turning_velocity 0.5
#define linear_velocity 0.5
#define maximum_speed 127

#define integral_gain 1
#define proportional_gain 1
#define Integral_Maximum 127
using namespace pros;

/*
F is forward
R ir turn right
L is turn left
B is go backwards
U is climb a pole
D is go down a pole
any integer values are the units

turning units is in degrees

Forward units is in meters

climbing units is in meters*/




//put auton indtructions here.




pros::Motor Left_Motor (left_motor_1, pros::v5::MotorGears::green);
//pros::Motor Left_Motor_2 (left_motor_2, pros::v5::MotorGears::blue);
//pros::Motor Left_Motor_3 (left_motor_3, pros::v5::MotorGears::blue);

pros::Motor Right_Motor (right_motor_1, pros::v5::MotorGears::green);
//pros::Motor Right_Motor_2 (right_motor_2, pros::v5::MotorGears::blue);
//pros::Motor Right_Motor_3 (right_motor_3, pros::v5::MotorGears::blue);

pros::Controller controller (E_CONTROLLER_MASTER);

pros::timer Timer;

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}	

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

double PID(const double err_val, const double delta_time, const double delta_err_val) {
	double integral = integral_gain * (Integral_Maximum * (err_val * delta_time));
	double proportional = proportional_gain * err_val;
	double derivative = proportional_gain * (delta_err_val / delta_time);
	return integral + proportional + derivative;
}



/*
Notes: 

we are making the robot run 3 loops around the thing for now. adding other features later.*/
void autonomous() {
	
}


void set_motor_voltage(const int linear_voltage, const int turning_voltage) {
	int8_t left_motor_voltage = ((linear_voltage^3 / maximum_speed^2) * linear_velocity) - ((turning_voltage^3 / maximum_speed^2) * turning_velocity);
	int8_t right_motor_voltage = ((linear_voltage^3 / maximum_speed^2) * linear_velocity) + ((turning_voltage^3 / maximum_speed^2) * turning_velocity);
	Left_Motor.move(left_motor_voltage);
	//Left_Motor_2.move(left_motor_voltage);
	//Left_Motor_3.move(left_motor_voltage);
	Right_Motor.move(right_motor_voltage);
	//Right_Motor_2.move(right_motor_voltage);
	//Right_Motor_3.move(right_motor_voltage);
}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	controller.print(0,0,"Program Started");
	while(true) {
		set_motor_voltage(controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X), controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
	}


}