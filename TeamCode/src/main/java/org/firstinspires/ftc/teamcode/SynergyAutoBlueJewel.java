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
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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


@Autonomous(name = "SynergyAutoBlueJewel", group = "Autonomous")
//@Disabled
public class SynergyAutoBlueJewel extends LinearOpMode {


    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159265);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    /**
     * The colorSensor field will contain a reference to our color sensor hardware object
     */
    NormalizedColorSensor colorSensor;
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

        //initialize the hardware map
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        //Reset robot encoders
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        //use encoders, when wheels are moving to keep at a certain speed
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :  %7d :%7d : %f",
                robot.leftRearMotor.getCurrentPosition(),
                robot.leftFrontMotor.getCurrentPosition(),
                robot.rightFrontMotor.getCurrentPosition(),
                robot.rightRearMotor.getCurrentPosition(),
                robot.jewelServo.getPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //move servo -90 to lower the arm with color sensor
        //note: servo move + makes the arm go down
        double servoInitPosition = robot.jewelServo.getPosition();
        robot.jewelServo.setPosition(servoInitPosition + 0.75);
        //sense ball color not yet created
        runtime.reset();
        double tmphsvValueZero = 0;
        while (runtime.seconds() < 5.0) {
            senseColor();
            telemetry.addLine()
                    .addData("H", "%.3f", hsvValues[0])
                    .addData("S", "%.3f", hsvValues[1])
                    .addData("V", "%.3f", hsvValues[2]);
            telemetry.update();
            tmphsvValueZero = Math.max(tmphsvValueZero, hsvValues[0]);
        }

        // (else, back 2 forward 2)

        if (tmphsvValueZero <= 300.0) {
            encoderDrive(DRIVE_SPEED, 0.1, 0.1, 0.1, 0.1, 3.0);
            robot.jewelServo.setPosition(servoInitPosition);
            //encoderDrive(DRIVE_SPEED, -0.1, -0.1, -0.1, -0.1, 3.0);
        } else {
            encoderDrive(DRIVE_SPEED, -0.1, -0.1, -0.1, -0.1, 3.0);
            robot.jewelServo.setPosition(servoInitPosition);
            //encoderDrive(DRIVE_SPEED, 0.1, 0.1, 0.1, 0.1, 3.0);

        }
        //move servo back to starting position
        //robot.jewelServo.setPosition(servoInitPosition);
        //encoderDrive(-DRIVE_SPEED, 9, 9, 9, 9, 3.0);
        runtime.reset();
        while (runtime.seconds() < 60.0) {

        }


        // Note: If Drive Speed in negative, the motors go forward
        //encoderDrive(-DRIVE_SPEED, -12.57, -12.57, -12.57, -0, 3.0);
        // move servo +90 to raise arm to starting position
        //robot.jewelServo.setPosition(servoInitPosition);
        //move robot back 2" or forward 2" to starting position depending on color
        //encoderDrive(DRIVE_SPEED, 2, 2, 2, 2, 3.0);
        //sense glyph image gKey= r,m,l
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //Note: For right lane
        //encoderDrive(-DRIVE_SPEED, 11, 11, 11, 11, 3.0);

        //encoderDrive(-TURN_SPEED,   12, -12, 12, -12, 4.0);
        //encoderDrive(-DRIVE_SPEED, 18.5, 18.5, 18.5, 18.5, 3.0);

        //Note: For middle lane
        //encoderDrive(-DRIVE_SPEED, 11, 11, 11, 11, 3.0);
        //encoderDrive(-DRIVE_SPEED,  16.5,  16.5, 16.5, 16.5, 3.0);
        //encoderDrive(-TURN_SPEED,   12, -12, 12, -12, 4.0);
        //encoderDrive(-DRIVE_SPEED, 18.5, 18.5, 18.5, 18.5, 3.0);

        //Mote: For left lane
        //encoderDrive(-DRIVE_SPEED, 11, 11, 11, 11, 3.0);
        //encoderDrive(-DRIVE_SPEED,  24, 24, 24, 24, 3.0);
        //encoderDrive(-TURN_SPEED,   12, -12, 12, -12, 4.0);
        //encoderDrive(-DRIVE_SPEED, 18.5, 18.5, 18.5, 18.5, 3.0);

        //encoderDrive(TURN_SPEED,   -2, 2,  4.0);
        // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   -12, 12, 4.0);  // S2: Turn Left 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, 10, 10, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout


        //robot.leftClaw.setPosition(1.0);
        //robot.rightClaw.setPosition(0.0);
        //stop moving for 1000 ticks
        sleep(1000);

        //show the drive path is completed on the driver station
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
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
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches,
                             double leftRearInches, double rightRearInches,
                             double timeoutS) throws InterruptedException {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
            newLeftRearTarget = robot.leftRearMotor.getCurrentPosition() + (int) (leftRearInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);
            newRightRearTarget = robot.rightRearMotor.getCurrentPosition() + (int) (rightRearInches * COUNTS_PER_INCH);

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.leftRearMotor.setTargetPosition(newLeftRearTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.rightRearMotor.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // keep looping while we are still active, and there is time left, and both motors are running.
            telemetry.addData("Path1", "Running at %7d :%7d %7d :%7d",
                    robot.leftFrontMotor.getCurrentPosition(),
                    robot.leftRearMotor.getCurrentPosition(),
                    robot.rightFrontMotor.getCurrentPosition(),
                    robot.rightRearMotor.getCurrentPosition());

            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.leftRearMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.rightRearMotor.setPower(Math.abs(speed));

            runtime.reset();
            while (runtime.seconds() < timeoutS) {

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.leftRearMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.rightRearMotor.setPower(0);

            telemetry.addData("Path2", "Running at %7d :%7d %7d :%7d",
                    robot.leftFrontMotor.getCurrentPosition(),
                    robot.leftRearMotor.getCurrentPosition(),
                    robot.rightFrontMotor.getCurrentPosition(),
                    robot.rightRearMotor.getCurrentPosition());

            telemetry.update();

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(5);   // optional pause after each move
        }
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
}





