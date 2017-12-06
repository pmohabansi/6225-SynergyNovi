package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Axis;

import java.util.Set;

/**
 * Created by alpay on 11/28/2017.
 */

@Autonomous(name="GyroTest", group = "Autonomous")
public class GyroTest extends LinearOpMode {
    /* Declare OpMode members. */
//    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device
    // Define variables for motors which are connected to the wheels to rotate.
    DcMotor leftFrontWheelMotor = null;
    DcMotor rightFrontWheelMotor = null;
    DcMotor leftRearWheelMotor = null;
    DcMotor rightRearWheelMotor = null;
    DcMotor armLiftMotor = null;
    Servo jewelServo = null;
    Servo leftArmMotor = null;
    Servo rightArmMotor = null;
    NormalizedColorSensor colorSensor;
    ModernRoboticsI2cGyro gyro = null;

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

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        leftFrontWheelMotor = hardwareMap.get(DcMotor.class, "lf");
        rightFrontWheelMotor = hardwareMap.get(DcMotor.class, "rf");
        leftRearWheelMotor = hardwareMap.get(DcMotor.class, "lr");
        rightRearWheelMotor = hardwareMap.get(DcMotor.class, "rr");
//        armLiftMotor = hardwareMap.dcMotor.get("al");
//        leftArmMotor = hardwareMap.servo.get("las");
//        rightArmMotor = hardwareMap.servo.get("ras");
        jewelServo = hardwareMap.get(Servo.class, "jewelServo");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontWheelMotor.setDirection(DcMotor.Direction.REVERSE);  //left of the robot (when facing away from robot)
        rightFrontWheelMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearWheelMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearWheelMotor.setDirection(DcMotor.Direction.FORWARD);

//        armLiftMotor.setDirection(DcMotor.Direction.FORWARD);
//        leftArmMotor.setDirection(Servo.Direction.FORWARD);
//        rightArmMotor.setDirection(Servo.Direction.FORWARD);

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

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        this.leftRearWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightRearWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
        }
        gyro.resetZAxisIntegrator();


        waitForStart();

        while (isStarted()) {
            telemetry.addData(">", "Robot Heading (running) = %d", gyro.getHeading());
            telemetry.update();
            sleep(300);
        }


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn

        //gyroDrive(DRIVE_SPEED, 18.0, 0.0);   // Drive FWD 18 inches
        //gyroTurn(TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
        //gyroDrive(DRIVE_SPEED, 6.0, 0.0);     //Drive FWD 6 inches

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distanceInches   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distanceInches,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     newLeftFrontTarget;
        int     newRightFrontTarget;
        int     newLeftRearTarget;
        int     newRightRearTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distanceInches * COUNTS_PER_INCH);
            newLeftFrontTarget = this.leftFrontWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);
            newLeftRearTarget = this.leftRearWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);
            newRightFrontTarget = this.rightFrontWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);
            newRightRearTarget = this.rightRearWheelMotor.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);
            // Set Target and Turn On RUN_TO_POSITION

            this.leftFrontWheelMotor.setTargetPosition(newLeftFrontTarget);
            this.leftRearWheelMotor.setTargetPosition(newLeftRearTarget);
            this.rightFrontWheelMotor.setTargetPosition(newRightFrontTarget);
            this.rightRearWheelMotor.setTargetPosition(newRightRearTarget);

            this.leftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.leftRearWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.rightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.rightRearWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            this.leftFrontWheelMotor.setPower(Math.abs(speed));
            this.leftRearWheelMotor.setPower(Math.abs(speed));
            this.rightFrontWheelMotor.setPower(Math.abs(speed));
            this.rightRearWheelMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (leftFrontWheelMotor.isBusy() || rightFrontWheelMotor.isBusy() ||
                    leftRearWheelMotor.isBusy() || rightFrontWheelMotor.isBusy())) {
                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distanceInches < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                this.leftFrontWheelMotor.setPower(Math.abs(leftSpeed));
                this.leftRearWheelMotor.setPower(Math.abs(leftSpeed));
                this.rightFrontWheelMotor.setPower(Math.abs(rightSpeed));
                this.rightRearWheelMotor.setPower(Math.abs(rightSpeed));

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Actual",  "%7d:%7d",      leftFrontWheelMotor.getCurrentPosition(),
                        rightFrontWheelMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            this.leftFrontWheelMotor.setPower(0);
            this.leftRearWheelMotor.setPower(0);
            this.rightFrontWheelMotor.setPower(0);
            this.rightRearWheelMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            this.leftRearWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.leftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.rightRearWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.rightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        this.leftFrontWheelMotor.setPower(0);
        this.leftRearWheelMotor.setPower(0);
        this.rightFrontWheelMotor.setPower(0);
        this.rightRearWheelMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        this.leftFrontWheelMotor.setPower(Math.abs(leftSpeed));
        this.leftRearWheelMotor.setPower(Math.abs(leftSpeed));
        this.rightFrontWheelMotor.setPower(Math.abs(rightSpeed));
        this.rightRearWheelMotor.setPower(Math.abs(rightSpeed));

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
