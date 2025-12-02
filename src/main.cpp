#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/colors.h"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"

bool logTemp = false;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// assets / file
ASSET(testpath_txt);

// motors
pros::MotorGroup left_drive_motors({1, -2, 3}, pros::MotorGears::blue);
pros::MotorGroup right_drive_motors({-6, 7, -8}, pros::MotorGears::blue);
pros::Motor outTakeMotor(14, pros::v5::MotorGears::blue);
pros::Motor intakeMotor(9, pros::v5::MotorGears::blue);
pros::adi::DigitalOut open_roof('A');

//optical
pros::Optical optical(10);

// position tracking
pros::Imu imu(11);
pros::Rotation horizontal_encoder(20);
pros::Rotation vertical_encoder(19);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_drive_motors, // left motor group
                              &right_drive_motors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              600, // drivetrain rpm is 600
                              2 // horizontal drift is 2 (for now)
);

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
lemlib::ControllerSettings angular_controller(10    , // proportional gain (kP)
                                              0, // integral gain (kI)
                                              200, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                            3 , // large error range, in degrees
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

// setup & initilization
void variable_init()
{
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    optical.set_led_pwm(100);
}

void selector() {
    // UI Interface for choosing autonomous routines.

    pros::screen_touch_status_s_t status = pros::screen::touch_status();
    int x = status.x;
    int y = status.y;

    // Draw objects to brain
    pros::screen::erase(); // removes frame smearing (vhs effect)
    pros::screen::set_pen(pros::c::COLOR_RED); // red alliance
    pros::screen::fill_rect(0, 0, 100, 100);
    pros::screen::fill_rect(110, 0, 210, 100);
    pros::screen::set_pen(pros::c::COLOR_BLUE); // blue alliance
    pros::screen::fill_rect(0, 110, 100, 210);
    pros::screen::fill_rect(110, 110, 210, 210);

    // Button checks to detect autonomous selection (what the player wants)
    if (status.press_count > 2)
    {
        if (x >= 0 && x <= 100 && y >= 0 && y <= 100) {
            pros::delay(300);
            chassis.follow(testpath_txt, 15, 2000);
        } 
        else if (x >= 110 && x <= 210 && y >= 0 && y <= 100) 
        {
            pros::delay(300);
            chassis.follow(testpath_txt, 15, 2000);
        }
        else if (x >= 0 && x <= 100 && y >= 110 && y <= 210) 
        {
            pros::delay(300);
            chassis.follow(testpath_txt, 15, 2000);
        } 
        else if (x >= 110 && x <= 210 && y >= 110 && y <= 210) {
            pros::delay(300);
            chassis.follow(testpath_txt, 15, 2000);
        } 
        else {
            left_drive_motors.move(127);
            right_drive_motors.move(127);

            pros::delay(200);
            left_drive_motors.move(0);
            right_drive_motors.move(0);
        }
        if (status.touch_status != TOUCH_PRESSED) {
            return;
        }
    }
}
void debug() {
    while (true) {

        if (logTemp)
        {
            printf("Optical Hue: %f\n", optical.get_hue());
            printf("Intake Temp: %f\n", intakeMotor.get_temperature());
            printf("Drivetrain Temp: %f\n", (left_drive_motors.get_temperature() + right_drive_motors.get_temperature()) / 2);
            pros::delay(500);
        }
    }
}
// initialize function. Runs on program startup
void initialize() {
    variable_init();
    pros::Task debugging_task(debug, "debuging");
    
    // run autonomous selector
    pros::Task screen_task([&]() {
        while (true){
            selector(); 
            pros::delay(20);
        }
         
    });
     pros::Task screen_task2([&]() {    
        while (true) {  
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Intake Temp: %f", intakeMotor.get_temperature());
            pros::lcd::print(4, "Outtake Temp: %f", outTakeMotor.get_temperature());
        // print measurements from the rotation sensor
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
        pros::delay(20);
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
 ASSET(testpath);
void autonomous() 
{   
    intakeMotor.move(127);
    chassis.follow(testpath_txt, 15, 2000);
    outTakeMotor.move(127);
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
    
        // move the robot
        chassis.arcade(-leftY, -rightX);
        
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intakeMotor.move(-127);
            outTakeMotor.move(-127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intakeMotor.move(127);
            outTakeMotor.move(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            intakeMotor.move(-127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intakeMotor.move(127);
        }
        else {
                if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) 
                && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) 
                {
                    intakeMotor.move(0);
                    outTakeMotor.move(0);
                }
            }
        
        if (logTemp)
        {
            if (left_drive_motors.get_temperature() > 50 || intakeMotor.get_temperature() > 50) {
                controller.clear();
                controller.rumble(".-");
                controller.set_text(1, 0, "Overheat!");
                pros::delay(100);
            }
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            open_roof.set_value(1);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            open_roof.set_value(0);
        }
        // delay to save resources
        pros::delay(20);
    }
}