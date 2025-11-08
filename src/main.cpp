#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/colors.h"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cstdio>
#include <atomic>

pros::Controller controller(pros::E_CONTROLLER_MASTER);
// left motor group
pros::MotorGroup left_motor_group({1, -2, 3}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({-6, 7, -8}, pros::MotorGears::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              600, // drivetrain rpm is 600
                              2 // horizontal drift is 2 (for now)
);
pros::adi::DigitalOut open_roof('A');
pros::adi::DigitalOut close_roof('B');
pros::Motor sorting_motor(14, pros::v5::MotorGears::blue);
// sorting motor
pros::Motor intake_motor(9, pros::v5::MotorGears::blue);
pros::Motor test(2);
// intake motor
//optical
pros::Optical optical(10);
// imu
pros::Imu imu(11);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(19);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
	10, // minimum output where drivetrain will move out of 127
	1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
 10, // minimum output where drivetrain will move out of 127
 1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
lateral_controller,
angular_controller,
sensors,
&throttle_curve, 
&steer_curve
);
std::atomic<int> g_pressed_rect{0};

void selector() {
    pros::screen_touch_status_s_t status = pros::screen::touch_status();
    int x = status.x;
    int y = status.y;
    pros::screen::set_pen(pros::c::COLOR_RED);
    pros::screen::fill_rect(0, 0, 100, 100);
    pros::screen::fill_rect(110, 0, 210, 100);
    pros::screen::set_pen(pros::c::COLOR_BLUE);
    pros::screen::fill_rect(0, 110, 100, 210);
    pros::screen::fill_rect(110, 110, 210, 210);
    if (x >= 0 && x <= 100 && y >= 0 && y <= 100) {
        g_pressed_rect.store(1); //red left
    } else if (x >= 110 && x <= 210 && y >= 0 && y <= 100) {
        g_pressed_rect.store(2); //red right
    } else if (x >= 0 && x <= 100 && y >= 110 && y <= 210) {
        g_pressed_rect.store(3); //blue left
    } else if (x >= 110 && x <= 210 && y >= 110 && y <= 210) {
        g_pressed_rect.store(4); //blue right
    } else {
        g_pressed_rect.store(0);
    }
    if (status.touch_status != TOUCH_PRESSED) {
        return;
    }
}
void debug() {
    while (true) {
        optical.set_led_pwm(100);
        printf("Optical Hue: %f\n", optical.get_hue());
        printf("Intake Temp: %f\n", intake_motor.get_temperature());
        printf("Drivetrain Temp: %f\n", (left_motor_group.get_temperature() + right_motor_group.get_temperature()) / 2);
        pros::delay(500);
    }
}
// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors  
    pros::Task debugging_task(debug, "debuging");
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true){
            selector(); 
            pros::delay(25);
        }
        
        while (true) {  
            // print robot location to the brain screen
            pros::lcd::print(4, "Optical Hue: %f", optical.get_hue()); // x
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        // print measurements from the rotation sensor
            pros::lcd::print(3, "Rotation Sensor: %i", optical.get_hue());
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    while (true){
            selector(); 
            pros::delay(25);
        }
}

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
void autonomous() 
{
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);
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
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        bool roof = false;
        int r = g_pressed_rect.load();
        optical.set_led_pwm(100);
        controller.print(1, 0, "Int Temp: %f", left_motor_group.get_temperature());
        // move the robot
        chassis.arcade(-leftY, -rightX);
        
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (r == 1 || r  == 2) { // red team
                if (optical.get_hue() > 65 && optical.get_hue() < 222) {
                    controller.set_text(0, 0, "blue");
                    sorting_motor.move(127);
                    intake_motor.move(-90);
                    pros::delay(600);
                }
                else if (optical.get_hue() >= 11 && optical.get_hue() <= 30) {
                    controller.set_text(0, 0, "red");
                    sorting_motor.move(-127);;
                    intake_motor.move(-90);
                    pros::delay(300);
                    intake_motor.move(0);
                }
                else {
                    //sorting_motor.move(-127);
                    intake_motor.move(-127);
                } 
            }
            else if (r == 3 || r == 4) { // blue team - perform the same actions as the previous block but targeted at red
                    if (optical.get_hue() >= 11 && optical.get_hue() <= 30) {
                        controller.set_text(0, 0, "red");
                        sorting_motor.move(127);
                        intake_motor.move(-90);
                        pros::delay(500);
                    }
                    else if (optical.get_hue() >= 65 && optical.get_hue() <= 222) {
                        controller.set_text(0, 0, "blue");
                        sorting_motor.move(-127);
                        intake_motor.move(-90);
                    }
                    else {
                        //sorting_motor.move(-127);
                        intake_motor.move(-127);
                    }
            }
        } // end of L1 code
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake_motor.move(127);
            sorting_motor.move(127);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            sorting_motor.move(127);
        } 
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            sorting_motor.move(-127);
        }
        else {
                if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) 
                && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) 
                && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) 
                {
                    intake_motor.move(0);
                    sorting_motor.move(0);
                }
            }
        if (left_motor_group.get_temperature() > 50 || intake_motor.get_temperature() > 50) {
            controller.clear();
            controller.rumble(".-");
            pros::delay(100);
            controller.set_text(1, 0, "Overheat!");
        }
        // delay to save resources
        pros::delay(25);
    }
}

