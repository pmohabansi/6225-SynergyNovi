/*
 * Developer: Apar Mohabansi
 * Date: 10/10/2017
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@SuppressWarnings({"WeakerAccess", "FieldCanBeLocal"})

@TeleOp(name = "Concept: TeleopDrive", group = "Concept")
public class TeleopDrive extends LinearOpMode {

    // Define variables for power to be given to the motors.
    double leftFrontWheelPower;
    double rightFrontWheelPower;
    double leftRearWheelPower;
    double rightRearWheelPower;
    int    armLiftNewPosition  = 0;
    double armLiftWheelPower   = 0.75;
    double wheelPowerLimit     = 0.75;
    double openArmPosition     = 0.7;
    double closeArmPosition    = 0.45;
    double openArmPositionSub  = 0.45;
    double closeArmPositionSub = 0.6;
    double jewelServoInitPosition;
    double leftArmOffset       = 0.1;

    static final double COUNTS_PER_MOTOR_REV  = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION  = 1.0;     // This is < 1.0 if geared UP
    static final double PULLEY_DIAMETER_INCHES = 1.05;   // For figuring circumference : used https://www.omnicalculator.com/math/circumference to convert circumference to diameter.
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                          (PULLEY_DIAMETER_INCHES * 3.1415);

    static final double PulleyThreadLength = 3; // actual complete string length is 47 inches.
    int armLiftPositionLimit = (int) (PulleyThreadLength * COUNTS_PER_INCH); // Get the absolute position and then the limit values;

    // Define variables for motors which are connected` to the wheels to rotate.
    DcMotor leftFrontWheelMotor  = null;
    DcMotor rightFrontWheelMotor = null;
    DcMotor leftRearWheelMotor   = null;
    DcMotor rightRearWheelMotor  = null;
    DcMotor armLiftMotor         = null;
    Servo   leftArmMotor         = null;
    Servo   rightArmMotor        = null;
    Servo   jewelServo           = null;

    // Declare LinearOpMode members.
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontWheelMotor  = hardwareMap.get(DcMotor.class, "lf");
        rightFrontWheelMotor = hardwareMap.get(DcMotor.class, "rf");
        leftRearWheelMotor   = hardwareMap.get(DcMotor.class, "lr");
        rightRearWheelMotor  = hardwareMap.get(DcMotor.class, "rr");
        armLiftMotor         = hardwareMap.dcMotor.get("al");
        leftArmMotor         = hardwareMap.servo.get("las");
        rightArmMotor        = hardwareMap.servo.get("ras");
        jewelServo           = hardwareMap.servo.get("jewelServo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontWheelMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontWheelMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearWheelMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearWheelMotor.setDirection(DcMotor.Direction.REVERSE);
        armLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftArmMotor.setDirection(Servo.Direction.FORWARD);
        rightArmMotor.setDirection(Servo.Direction.REVERSE);
        jewelServo.setDirection(Servo.Direction.FORWARD);

        leftRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        jewelServoInitPosition = jewelServo.getPosition();
        leftArmMotor.setPosition(openArmPosition-leftArmOffset);
        rightArmMotor.setPosition(openArmPosition);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        waitForStart();

        while (opModeIsActive()) {
            jewelServo.setPosition(jewelServoInitPosition); // hold the position.

            // Initialize power to zero so in case user is not pressing any keys then
            // robot should remain in same position.
            leftFrontWheelPower = 0;
            rightFrontWheelPower = 0;
            leftRearWheelPower = 0;
            rightRearWheelPower = 0;

            // calculated power to be given to wheels
            // if power value is -ve then robot forward &
            // when power value is +ve then robot backward
            if (gamepad1.right_stick_y != 0) {
                // This is for moving the robot forward and reverse
                telemetry.addLine("forward/back");

                // When Y is moved upward then system receive -ve value
                // & when Y is moved down then system receive +ve value.

                leftFrontWheelPower = Range.clip(gamepad1.right_stick_y, -wheelPowerLimit, wheelPowerLimit);
                rightFrontWheelPower = Range.clip(gamepad1.right_stick_y, -wheelPowerLimit, wheelPowerLimit);
                leftRearWheelPower = Range.clip(gamepad1.right_stick_y, -wheelPowerLimit, wheelPowerLimit);
                rightRearWheelPower = Range.clip(gamepad1.right_stick_y, -wheelPowerLimit, wheelPowerLimit);
            } else if (gamepad1.right_stick_x != 0) {
                // This is for turning the robot right and left
                telemetry.addLine("turning");

                // Similarly when X is moved left then system receive -ve value
                // & when X is moved right then system receive +ve value.

                leftFrontWheelPower = Range.clip(-gamepad1.right_stick_x, -wheelPowerLimit, wheelPowerLimit);
                rightFrontWheelPower = Range.clip(gamepad1.right_stick_x, -wheelPowerLimit, wheelPowerLimit);
                leftRearWheelPower = Range.clip(-gamepad1.right_stick_x, -wheelPowerLimit, wheelPowerLimit);
                rightRearWheelPower = Range.clip(gamepad1.right_stick_x, -wheelPowerLimit, wheelPowerLimit);
            } else if (gamepad1.right_trigger != 0) {
                // This is for shifting the robot to the right
                telemetry.addLine("shifting right");

                leftFrontWheelPower = Range.clip(-gamepad1.right_trigger, -wheelPowerLimit, wheelPowerLimit);
                rightFrontWheelPower = Range.clip(gamepad1.right_trigger, -wheelPowerLimit, wheelPowerLimit);
                leftRearWheelPower = Range.clip(gamepad1.right_trigger, -wheelPowerLimit, wheelPowerLimit);
                rightRearWheelPower = Range.clip(-gamepad1.right_trigger, -wheelPowerLimit, wheelPowerLimit);
            } else if (gamepad1.left_trigger != 0) {
                // This is for shifting the robot to the left
                telemetry.addLine("shifting left");

                leftFrontWheelPower = Range.clip(gamepad1.left_trigger, -wheelPowerLimit, wheelPowerLimit);
                rightFrontWheelPower = Range.clip(-gamepad1.left_trigger, -wheelPowerLimit, wheelPowerLimit);
                leftRearWheelPower = Range.clip(-gamepad1.left_trigger, -wheelPowerLimit, wheelPowerLimit);
                rightRearWheelPower = Range.clip(gamepad1.left_trigger, -wheelPowerLimit, wheelPowerLimit);

            } else if ((gamepad1.left_stick_x > 0) && (gamepad1.left_stick_y < 0)) {
                // This is for moving the robot to the diagonal forward right
                telemetry.addLine("diagonal forward right");

                leftFrontWheelPower = -wheelPowerLimit;
                rightFrontWheelPower = 0;
                leftRearWheelPower = 0;
                rightRearWheelPower = -wheelPowerLimit;
            } else if ((gamepad1.left_stick_x < 0) && (gamepad1.left_stick_y > 0)) {
                // This is for moving the robot to the diagonal backward left
                telemetry.addLine("diagonal backward left");

                leftFrontWheelPower = wheelPowerLimit;
                rightFrontWheelPower = 0;
                leftRearWheelPower = 0;
                rightRearWheelPower = wheelPowerLimit;
            } else if ((gamepad1.left_stick_x < 0) && (gamepad1.left_stick_y < 0)) {
                // This is for moving the robot to the diagonal forward left
                telemetry.addLine("diagonal forward left");

                leftFrontWheelPower = 0;
                rightFrontWheelPower = -wheelPowerLimit;
                leftRearWheelPower = -wheelPowerLimit;
                rightRearWheelPower = 0;
            } else if ((gamepad1.left_stick_x > 0) && (gamepad1.left_stick_y > 0)) {
                // This is for moving the robot to the diagonal backward right
                telemetry.addLine("diagonal backward right");

                leftFrontWheelPower = 0;
                rightFrontWheelPower = wheelPowerLimit;
                leftRearWheelPower = wheelPowerLimit;
                rightRearWheelPower = 0;
            }

            if (gamepad2.right_stick_y != 0) {
                // This is to lift arm
                if(gamepad2.right_stick_y < 0) {
                    armLiftNewPosition += ((int)(1 * COUNTS_PER_INCH))*-1;
                } else if(gamepad2.right_stick_y > 0) {
                    armLiftNewPosition += ((int)(1 * COUNTS_PER_INCH));
                }

                armLiftNewPosition = Range.clip(armLiftNewPosition, 0, armLiftPositionLimit);

                if((armLiftNewPosition > 0) &&
                        (armLiftNewPosition < armLiftPositionLimit)) {
                    armLiftMotor.setTargetPosition(armLiftNewPosition);
                    armLiftMotor.setPower(armLiftWheelPower); // We might need to adjust in case we observe a jerk in the movement.

                    // Allow the system to work on rotating the arm lift motor to get to the new position.
                    while (opModeIsActive() && armLiftMotor.isBusy()) {
                        telemetry.addData("Status", "Run Time: " + runtime.toString());
                        telemetry.addData("ArmLiftMotor", "Inner NewPos(%7d) LimitPos(%7d)",
                                armLiftNewPosition,
                                armLiftPositionLimit
                        );
                        telemetry.update();
                        idle();
                    }
                } else {
                    telemetry.addData("ArmLiftMotor", "armLiftNewPosition == armLiftPositionLimit");
                    telemetry.update();
                    armLiftMotor.setPower(0);
                }
            }

            // no "else if" will allow to lift arm as well as grab the glyph(s)
            if (gamepad2.right_trigger != 0) {
                // This is for opening the arms
                leftArmMotor.setPosition(closeArmPosition);
                rightArmMotor.setPosition(closeArmPosition);
                telemetry.addData("left Arm Position", leftArmMotor.getPosition());
                telemetry.addData("right Arm Position", rightArmMotor.getPosition());
            } else if (gamepad2.left_trigger != 0) {
                // This is for opening the arms
                leftArmMotor.setPosition(openArmPosition-leftArmOffset);
                rightArmMotor.setPosition(openArmPosition);
                telemetry.addData("left Arm Position", leftArmMotor.getPosition());
                telemetry.addData("right Arm Position", rightArmMotor.getPosition());
            } else if ( gamepad2.left_bumper ) {
                // This is for close the arms
                leftArmMotor.setPosition(closeArmPositionSub);
                rightArmMotor.setPosition(closeArmPositionSub);
                telemetry.addData("left Arm Position", leftArmMotor.getPosition());
                telemetry.addData("right Arm Position", rightArmMotor.getPosition());
            } else if ( gamepad2.right_bumper ) {
                // This is for opening the arms
                leftArmMotor.setPosition(openArmPositionSub);
                rightArmMotor.setPosition(openArmPositionSub);
                telemetry.addData("left Arm Position", leftArmMotor.getPosition());
                telemetry.addData("right Arm Position", rightArmMotor.getPosition());
            }

            telemetry.addLine("");
            // Send calculated power to wheels
            leftFrontWheelMotor.setPower(leftFrontWheelPower);
            rightFrontWheelMotor.setPower(rightFrontWheelPower);
            leftRearWheelMotor.setPower(leftRearWheelPower);
            rightRearWheelMotor.setPower(rightRearWheelPower);

            if(gamepad2.right_stick_y == 0){
                armLiftMotor.setPower(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "front left (%.2f), front right (%.2f)," +
                            " rear left (%.2f), rear right (%.2f).",
                    leftFrontWheelPower,
                    rightFrontWheelPower,
                    leftRearWheelPower,
                    rightRearWheelPower
            );
            telemetry.addData("ArmLiftMotor", "Arm Lift NewPos(%7d) LimitPos(%7d)",
                    armLiftNewPosition,
                    armLiftPositionLimit
            );

            telemetry.update();
        }
    }
}
