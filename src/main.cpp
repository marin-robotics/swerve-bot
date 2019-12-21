#include "main.h"
#include <cmath>

#define PI 3.14159265

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor lift_mtr_0(4, true);
pros::Motor lift_mtr_1(5);
pros::Motor claw_mtr_0(1);
pros::Motor claw_mtr_1(2, true);
pros::Motor front_right_mtr(6);
pros::Motor front_left_mtr(9);
pros::Motor rear_right_mtr(7, true);
pros::Motor rear_left_mtr(8, true);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();

	pros::lcd::register_btn1_cb(on_center_button);
}

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
void autonomous() {
	// move forward
	front_right_mtr = -70;
	rear_left_mtr = -70;
	rear_right_mtr = 70;
	front_left_mtr = 70;
	pros::delay(600);
	// move backwards
/*	front_right_mtr = -90;
	rear_left_mtr = -90;
	rear_right_mtr = 90;
	front_left_mtr = 90;*/
	// stop movement
	front_right_mtr = 0;
	rear_left_mtr = 0;
	rear_right_mtr = 0;
	front_left_mtr = 0;
	// deploy lift
	lift_mtr_0 = 70;
	lift_mtr_1 = 70;
	pros::delay(3000);
	lift_mtr_0 = 0;
	lift_mtr_1 = 0;
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
	while (true) {
		double x = master.get_analog(ANALOG_RIGHT_X);
		double y = master.get_analog(ANALOG_RIGHT_Y);
		double left_x = master.get_analog(ANALOG_LEFT_X);

		// Movement

		double ratio = atan2(y, x) + PI/4;
		x = sqrt(x*x + y*y) * cos(ratio);
		y = sqrt(x*x + y*y) * sin(ratio);

		front_right_mtr = x + left_x;
		rear_left_mtr = x - left_x;
		rear_right_mtr = y - left_x;
		front_left_mtr = y + left_x;

		// Lifting

		int analog_lift_power = master.get_analog(ANALOG_LEFT_Y);
		lift_mtr_0 = -analog_lift_power;
		lift_mtr_1 = -analog_lift_power;

		if (master.get_digital(DIGITAL_UP)) {
			lift_mtr_0 = 32;
			lift_mtr_1 = 32;
		} else if (master.get_digital(DIGITAL_DOWN)) {
			lift_mtr_0 = -32;
			lift_mtr_1 = -32;
		}

		if (master.get_digital(DIGITAL_LEFT)){
			front_right_mtr = -52;
			front_left_mtr = -52;
			rear_right_mtr = 52;
			rear_left_mtr = 52;
		} else if (master.get_digital(DIGITAL_RIGHT)) {
			front_left_mtr = 52;
			front_right_mtr = 52;
			rear_left_mtr = -52;
			rear_right_mtr = -52;
		}

		if (master.get_digital(DIGITAL_L1)) {
			claw_mtr_0 = 127;
			claw_mtr_1 = 127;
		} else if (master.get_digital(DIGITAL_R1)) {
			claw_mtr_0 = -127;
			claw_mtr_1 = -127;
		} else if (master.get_digital(DIGITAL_L2)) {
			claw_mtr_1 = 33;
			claw_mtr_0 = 33;
		} else if (master.get_digital(DIGITAL_R2)) {
			claw_mtr_1 = -33;
			claw_mtr_0 = -33;
		} else {
			claw_mtr_0 = 0;
			claw_mtr_1 = 0;
		}

		pros::delay(20);
	}
}
