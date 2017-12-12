/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND " AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "BlueLeftTest", group = "Autonomous")
//@Disabled
public class BlueLeftTest extends LinearOpMode {
    // Define variables for motors which are connected to the wheels to rotate.
    DcMotor leftFrontWheelMotor = null;
    DcMotor rightFrontWheelMotor = null;
    DcMotor leftRearWheelMotor = null;
    DcMotor rightRearWheelMotor = null;
    DcMotor armLiftMotor = null;
    Servo   jewelServo = null;
    Servo   leftArmMotor = null;
    Servo   rightArmMotor = null;
    ModernRoboticsI2cGyro gyro    = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    static final double COUNTS_PER_MOTOR_REV  = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION  = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED  = 0.4;
    static final double DISTANCE_IN_INCHES_PER_SECOND = 30.75;    // at power 0.4

    /**
     * The colorSensor field will contain a reference to our color sensor hardware object
     */
    NormalizedColorSensor colorSensor;
    RelicRecoveryVuMark vuMark1;
    /**
     * The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need something analogous when you
     * use a color sensor on your robot
     */
    View relativeLayout;
    float[] hsvValues = new float[3];
    int colorValue;

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;
    RelicRecoveryVuMark vuMarkIdentification = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontWheelMotor = hardwareMap.get(DcMotor.class, "lf");
        rightFrontWheelMotor = hardwareMap.get(DcMotor.class, "rf");
        leftRearWheelMotor = hardwareMap.get(DcMotor.class, "lr");
        rightRearWheelMotor = hardwareMap.get(DcMotor.class, "rr");
        armLiftMotor = hardwareMap.dcMotor.get("al");
        leftArmMotor = hardwareMap.servo.get("las");
        rightArmMotor = hardwareMap.servo.get("ras");
        jewelServo = hardwareMap.get(Servo.class, "jewelServo");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontWheelMotor.setDirection(DcMotor.Direction.REVERSE);  //left of the robot (when facing away from robot)
        rightFrontWheelMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearWheelMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearWheelMotor.setDirection(DcMotor.Direction.FORWARD);

        armLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftArmMotor.setDirection(Servo.Direction.FORWARD);
        rightArmMotor.setDirection(Servo.Direction.REVERSE);

        leftFrontWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Zero out the encoders
        leftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;


        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();


        this.leftRearWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightRearWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :  %7d :%7d : %f",
                this.leftRearWheelMotor.getCurrentPosition(),
                this.leftFrontWheelMotor.getCurrentPosition(),
                this.rightFrontWheelMotor.getCurrentPosition(),
                this.rightRearWheelMotor.getCurrentPosition(),
                this.jewelServo.getPosition());
        telemetry.update();

           /*
        * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
        * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
        */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AToaQHT/////AAAAGVlEmfHGCEBvpO9+LDtLj0l8DvOeohU4FwBvMiWCfaZuiAhtRFKLnku1xVmWpr1AyFGJ3vufY4HpayP44D5ZSmVg6pxonVjXN4HULK10Z3sA8zXdGbHccIkVp0h7teeFC7OyI3cy/+/I6scuRBWgUYjP0300xAD80dfV/Z5ra8MTHW3OuLyd4hpC/EMP27Af5a+M+Z2+DaEYw78jGKAW335HsHRC45E65I2ozwtIEwnJDeCvHD4ulFnC0v/+4kUDynCAKBd9eNvidbixN6nWN7ltOYFf4TwTYNLMKNgIex0zyGUjRfabJGugkCgB2dKbKrNzMYK2oLOqODgcaI7lHdkGosf00X1X3yduZOuh/ccY";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
//        while (!isStarted()) {
//            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
//            telemetry.update();
//        }
//
//        gyro.resetZAxisIntegrator();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        leftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.leftRearWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightRearWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //note: servo move + makes the arm go down
        //grip glyph in arm
        double leftArmInitPosition = this.leftArmMotor.getPosition();
        double rightArmInitPosition = this.rightArmMotor.getPosition();

        telemetry.addData("Path0", "RightArm LeftArm %f %f",
                this.rightArmMotor.getPosition(),
                this.leftArmMotor.getPosition());
        telemetry.update();
        sleep(1000);
        this.rightArmMotor.setPosition(0.3);
        this.leftArmMotor.setPosition(0.4);

        //lift glyph inside the arms of the glyph attatchment
        this.armLiftMotor.setPower(-0.3);
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            idle();
        }
        telemetry.addData("Path0", "RightArm LeftArm %f %f",
                this.rightArmMotor.getPosition(),
                this.leftArmMotor.getPosition());
        telemetry.update();
        sleep(1000);
        this.armLiftMotor.setPower(0);

        //sense the pictograph
        sensePictograph();
        telemetry.addData("VuMar1k", "%s visible", vuMark1);
        telemetry.update();
        //lower jewelArm to sense color
        double servoInitPosition = this.jewelServo.getPosition();
        this.jewelServo.setPosition(servoInitPosition + 0.8);
        //sense ball color
        runtime.reset();
        double tmphsvValueZero = 0;
        while (runtime.seconds() < 2.0) {
            senseColor();
            telemetry.addLine()
                    .addData("H", "%.3f", hsvValues[0])
                    .addData("S", "%.3f", hsvValues[1])
                    .addData("V", "%.3f", hsvValues[2]);
            telemetry.update();
            tmphsvValueZero = Math.max(tmphsvValueZero, hsvValues[0]);
        }
        //inches from starting point, to right column
        double distanceToRightColumn = -28;
        double distanceToColumn = 0;
        //move forward if red, backwards if blue
        if (tmphsvValueZero <= 300.0 && tmphsvValueZero >= 120.0) {
            encoderDrive(DRIVE_SPEED, 0, -5.0);
            //sleep(1000);
            this.jewelServo.setPosition(servoInitPosition);
            while (runtime.seconds() < 1.0) {
                idle();
            }
            //sleep(1000);
            if (vuMark1.toString().toUpperCase().contains("RIGHT")) {
                distanceToColumn = distanceToRightColumn +5;
                encoderDrive(DRIVE_SPEED, 0, distanceToColumn);
                encoderDrive(DRIVE_SPEED, -2, 16);
                encoderDrive(TURN_SPEED, -1, 13);
            } else if (vuMark1.toString().toUpperCase().contains("CENTER")) {
                distanceToColumn = distanceToRightColumn +5;
                encoderDrive(DRIVE_SPEED, 0, distanceToColumn);
                encoderDrive(DRIVE_SPEED, -2, 8 );
                encoderDrive(TURN_SPEED, -1, 13);
            } else {
                distanceToColumn = distanceToRightColumn +5;
                encoderDrive(DRIVE_SPEED, 0, distanceToColumn);
                encoderDrive(TURN_SPEED, -1, 13);
            }
        } else {
            encoderDrive(DRIVE_SPEED, 0, 5.0);
            //sleep(1000);
            this.jewelServo.setPosition(servoInitPosition);
            while (runtime.seconds() < 1.0) {
                idle();
            }
            //sleep(1000);
            if (vuMark1.toString().toUpperCase().contains("LEFT")) {
                distanceToColumn = distanceToRightColumn - 10;
                encoderDrive(DRIVE_SPEED, 0, distanceToColumn);
                encoderDrive(DRIVE_SPEED, -2, 16);
                encoderDrive(TURN_SPEED, -1, 13);
            } else if (vuMark1.toString().toUpperCase().contains("CENTER")) {
                distanceToColumn = distanceToRightColumn - 10;
                encoderDrive(DRIVE_SPEED, 0, distanceToColumn);
                encoderDrive(DRIVE_SPEED, -2, 8 );
                encoderDrive(TURN_SPEED, -1, 13);
            } else {
                distanceToColumn = distanceToRightColumn - 10;
                encoderDrive(DRIVE_SPEED, 0, distanceToColumn);
                encoderDrive(TURN_SPEED, -1, 13);
            }
        }
        //sleep(1000);
        //turn to cryptobox
//        encoderDrive(DRIVE_SPEED, -1, distanceToColumn);
//        //move forward to cryptobox position
        encoderDrive(DRIVE_SPEED, 0, 10);

        //open glyph arm
        this.leftArmMotor.setPosition(0.55);
        this.rightArmMotor.setPosition(0.65);
        //drive back
        encoderDrive(DRIVE_SPEED, 0, -3);
        this.armLiftMotor.setPower(0.3);
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            idle();
        }
        telemetry.addData("Path0", "RightArm LeftArm %f %f",
                this.rightArmMotor.getPosition(),
                this.leftArmMotor.getPosition());
        telemetry.update();
        sleep(1000);
        this.armLiftMotor.setPower(0);

        encoderDrive(DRIVE_SPEED, 0, 4);
        encoderDrive(DRIVE_SPEED, 0, -5);
        encoderDrive(DRIVE_SPEED, -2, 3);

//        this.leftArmMotor.setPosition(0.0);
//        this.rightArmMotor.setPosition(-0.0);
//        encoderDrive(DRIVE_SPEED, 0, 6.25);
//        //drive back
//        encoderDrive(DRIVE_SPEED, 0, -2);
        /*
        encoderDrive(DRIVE_SPEED, -1, 20);
        double safeZoneDist = 7;
        if (vuMark1.toString().toUpperCase().contains("LEFT")) {
            safeZoneDist = -safeZoneDist;
        } else if (vuMark1.toString().toUpperCase().contains("CENTER")) {
            safeZoneDist = 0;
        }
        encoderDrive(DRIVE_SPEED, 0, safeZoneDist);
        //glide right into safezone
        encoderDrive(DRIVE_SPEED, 2, 8);

        */
    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    //gives the programing for encoder drive
    public void encoderDrive(double speed, int drive_mode, double distanceInches) throws InterruptedException {
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftRearTarget = 0;
        int newRightRearTarget = 0;

        leftFrontWheelMotor.setDirection(DcMotor.Direction.REVERSE);  //left of the robot (when facing away from robot)
        rightFrontWheelMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearWheelMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearWheelMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //use encoders, when wheels are moving to keep at a certain speed
        this.leftRearWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightRearWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        this.leftFrontWheelMotor.setTargetPosition(this.leftFrontWheelMotor.getCurrentPosition());
//        this.rightFrontWheelMotor.setTargetPosition(this.rightFrontWheelMotor.getCurrentPosition());
//        this.leftRearWheelMotor.setTargetPosition(this.leftRearWheelMotor.getCurrentPosition());
//        this.rightRearWheelMotor.setTargetPosition(this.rightRearWheelMotor.getCurrentPosition());

//        this.leftFrontWheelMotor.setTargetPosition(0);
//        this.rightFrontWheelMotor.setTargetPosition(0);
//        this.leftRearWheelMotor.setTargetPosition(0);
//        this.rightRearWheelMotor.setTargetPosition(0);

        this.leftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftRearWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightRearWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        while (opModeIsActive() && (leftFrontWheelMotor.isBusy() || rightFrontWheelMotor.isBusy() ||
//                leftRearWheelMotor.isBusy() || rightFrontWheelMotor.isBusy())) {
//            telemetry.addData("Status1", "Run Time: " + runtime.toString());
//            telemetry.addData("Path01", "Running at %7d :%7d %7d :%7d",
//                    leftFrontWheelMotor.getCurrentPosition(),
//                    leftRearWheelMotor.getCurrentPosition(),
//                    rightFrontWheelMotor.getCurrentPosition(),
//                    rightRearWheelMotor.getCurrentPosition());
//            telemetry.update();
//        }

        //sleep(5000);

        //drive_mode==0 means, move straight
        //if drive_mode==-1 means, turn left
        //if drive_mode==1 means, turn right
        if (drive_mode == 0) {
            // Determine new target position, and pass to motor controller
            //telemetry.addLine("move straight");
            //telemetry.update();
            newLeftFrontTarget = this.leftFrontWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);
            newLeftRearTarget = this.leftRearWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);
            newRightFrontTarget = this.rightFrontWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);
            newRightRearTarget = this.rightRearWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);

            telemetry.addData("drive_mode 0", "Running at %7d :%7d %7d :%7d : %7d :%7d %7d :%7d",
                    newLeftFrontTarget,
                    newLeftRearTarget,
                    newRightFrontTarget,
                    newRightRearTarget,
                    this.leftFrontWheelMotor.getCurrentPosition(),
                    this.leftRearWheelMotor.getCurrentPosition(),
                    this.rightFrontWheelMotor.getCurrentPosition(),
                    this.rightRearWheelMotor.getCurrentPosition());
            telemetry.update();
        } else if (drive_mode == -1) {
            // Determine new target position, and pass to motor controller
            telemetry.addLine("turn left");
            telemetry.update();
            newLeftFrontTarget = this.leftFrontWheelMotor.getCurrentPosition() + (int) (-distanceInches * COUNTS_PER_INCH);
            newLeftRearTarget = this.leftRearWheelMotor.getCurrentPosition() + (int) (-distanceInches * COUNTS_PER_INCH);
            newRightFrontTarget = this.rightFrontWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);
            newRightRearTarget = this.rightRearWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);
        } else if (drive_mode == 1) {
            // Determine new target position, and pass to motor controller
            telemetry.addLine("turn right");
            telemetry.update();
            newLeftFrontTarget = this.leftFrontWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);
            newLeftRearTarget = this.leftRearWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);
            newRightFrontTarget = this.rightFrontWheelMotor.getCurrentPosition() + (int) (-distanceInches * COUNTS_PER_INCH);
            newRightRearTarget = this.rightRearWheelMotor.getCurrentPosition() + (int) (-distanceInches * COUNTS_PER_INCH);
        } else if (drive_mode == 2) {
            // Determine new target position, and pass to motor controller
            telemetry.addLine("glide right");
            telemetry.update();
            newLeftFrontTarget = this.leftFrontWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);
            newLeftRearTarget = this.leftRearWheelMotor.getCurrentPosition() + (int) (-distanceInches * COUNTS_PER_INCH);
            newRightFrontTarget = this.rightFrontWheelMotor.getCurrentPosition() + (int) (-distanceInches * COUNTS_PER_INCH);
            newRightRearTarget = this.rightRearWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);
        } else if (drive_mode == -2) {
            // Determine new target position, and pass to motor controller
            telemetry.addLine("glide left");
            telemetry.update();
            newLeftFrontTarget = this.leftFrontWheelMotor.getCurrentPosition() + (int) (-distanceInches * COUNTS_PER_INCH);
            newLeftRearTarget = this.leftRearWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);
            newRightFrontTarget = this.rightFrontWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);
            newRightRearTarget = this.rightRearWheelMotor.getCurrentPosition() + (int) (-distanceInches * COUNTS_PER_INCH);
        }

        //sleep(5000);

        this.leftFrontWheelMotor.setTargetPosition(newLeftFrontTarget);
        this.leftRearWheelMotor.setTargetPosition(newLeftRearTarget);
        this.rightFrontWheelMotor.setTargetPosition(newRightFrontTarget);
        this.rightRearWheelMotor.setTargetPosition(newRightRearTarget);

        this.leftFrontWheelMotor.setPower(Math.abs(speed));
        this.leftRearWheelMotor.setPower(Math.abs(speed));
        this.rightFrontWheelMotor.setPower(Math.abs(speed));
        this.rightRearWheelMotor.setPower(Math.abs(speed));

        while (opModeIsActive() && (leftFrontWheelMotor.isBusy() || rightFrontWheelMotor.isBusy() ||
                leftRearWheelMotor.isBusy() || rightFrontWheelMotor.isBusy())) {
            telemetry.addData("Status2", "Run Time: " + runtime.toString());
            telemetry.addData("Path2", "Running at %7d :%7d %7d :%7d : %7d :%7d %7d :%7d",
                    newLeftFrontTarget,
                    newLeftRearTarget,
                    newRightFrontTarget,
                    newRightRearTarget,
                    leftFrontWheelMotor.getCurrentPosition(),
                    leftRearWheelMotor.getCurrentPosition(),
                    rightFrontWheelMotor.getCurrentPosition(),
                    rightRearWheelMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
        //sleep(5000);   // optional pause after each move
        // Stop all motion;
        this.leftFrontWheelMotor.setPower(0);
        this.leftRearWheelMotor.setPower(0);
        this.rightFrontWheelMotor.setPower(0);
        this.rightRearWheelMotor.setPower(0);
        //sleep(5000);   // optional pause after each move

        this.leftRearWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightRearWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void senseColor() throws InterruptedException {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Get a reference to the RelativeLayout so we can later change the background
            // color of the Robot Controller app to match the hue detected by the RGB sensor.
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
            // values is a reference to the hsvValues array.
            final float values[] = hsvValues;


            // Get a reference to our sensor object.
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
            // If possible, turn the light on in the beginning (it might already be on anyway,
            // we just make sure it is if we can).
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight) colorSensor).enableLight(false);
            }
            if (colorSensor instanceof SwitchableLight) {
                SwitchableLight light = (SwitchableLight) colorSensor;
                light.enableLight(!light.isLightOn());
            }

            // Read the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            /** Use telemetry to display feedback on the driver station. We show the conversion
             * of the colors to hue, saturation and value, and display the the normalized values
             * as returned from the sensor.
             * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/

            Color.colorToHSV(colors.toColor(), hsvValues);
//            telemetry.addLine()
//                    .addData("H", "%.3f", hsvValues[0])
//                    .addData("S", "%.3f", hsvValues[1])
//                    .addData("V", "%.3f", hsvValues[2]);
//            telemetry.addLine()
//                    .addData("a", "%.3f", colors.alpha)
//                    .addData("r", "%.3f", colors.red)
//                    .addData("g", "%.3f", colors.green)
//                    .addData("b", "%.3f", colors.blue);
/** We also display a conversion of the colors to an equivalent Android color integer.
 * @see Color */
            int color = colors.toColor();
            colorValue = color;
//            telemetry.addLine("raw Android color: ")
//                    .addData("a", "%02x", Color.alpha(color))
//                    .addData("r", "%02x", Color.red(color))
//                    .addData("g", "%02x", Color.green(color))
//                    .addData("b", "%02x", Color.blue(color));

            // Balance the colors. The values returned by getColors() are normalized relative to the
            // maximum possible values that the sensor can measure. For example, a sensor might in a
            // particular configuration be able to internally measure color intensity in a range of
            // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
            // so as to return a value it the range [0,1]. However, and this is the point, even so, the
            // values we see here may not get close to 1.0 in, e.g., low light conditions where the
            // sensor measurements don't approach their maximum limit. In such situations, the *relative*
            // intensities of the colors are likely what is most interesting. Here, for example, we boost
            // the signal on the colors while maintaining their relative balance so as to give more
            // vibrant visual feedback on the robot controller visual display.
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

//            telemetry.addLine("normalized color:  ")
//                    .addData("a", "%02x", Color.alpha(color))
//                    .addData("r", "%02x", Color.red(color))
//                    .addData("g", "%02x", Color.green(color))
//                    .addData("b", "%02x", Color.blue(color));
//            telemetry.update();

            // convert the RGB values to HSV values.
            Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });
        }
    }

    public void sensePictograph() throws InterruptedException {

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        relicTrackables.activate();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.0) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            vuMark1 = vuMark;
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }

    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}






