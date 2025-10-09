#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/misc.h"
#include <cstdio>

pros::Controller controller(pros::E_CONTROLLER_MASTER);
// left motor group
pros::MotorGroup left_motor_group({1, 2, -3}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({-6, -7, 8}, pros::MotorGears::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              13, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              400, // drivetrain rpm is 600
                              2 // horizontal drift is 2 (for now)
);
//optical
pros::Optical optical(9);
// imu
pros::Imu imu(10);
// horizontal tracking wheel encoder
pros::adi::Encoder horizontal_encoder('A', 'B', false);
// vertical tracking wheel encoder
pros::adi::Encoder vertical_encoder('C', 'D', true);
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
//debug task to print alot of info that we are curious about.
void debug() {
    while(true){
        std::cout << "Battery level:" << controller.get_battery_level()  << std::endl;
        std::cout << "CONTROLLER LEFT Y:" << controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)  << std::endl;
        std::cout << "button test" << controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) << std::endl;

        pros::delay(1000);
    }
}

// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task debug_task(debug);
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(4, "Optical Hue: %f", optical.get_hue()); // x
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        // print measurements from the rotation sensor
            pros::lcd::print(3, "Rotation Sensor: %f", imu.get_rotation()); // heading
            int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            controller.set_text(0,0, std::to_string(leftY));
            
            // delay to save resources
            pros::delay(100);
        }
    });
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
ASSET(example_txt);
void autonomous() 
{
    chassis.setPose(lemlib::Pose(-50, -40, 90));
    chassis.moveToPoint(10, 10, 4000);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediatelyoller.set_text(0,1, std::to_string(rightX));
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
        // get left y and right x positions
        left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        //printf(right_motor_group.get_efficiency_all());
        // move the robot
        chassis.arcade(leftY, rightX);

        //std::cout << "Output" << std::endl;
        
        


        // delay to save resources
        pros::delay(25);
    }
}
