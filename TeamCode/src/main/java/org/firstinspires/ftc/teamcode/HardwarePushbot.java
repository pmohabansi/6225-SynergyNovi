package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwarePushbot
{
    /* Public OpMode members. */

    public DcMotor  leftFrontMotor   = null;
    public DcMotor  rightFrontMotor  = null;
    public DcMotor  leftRearMotor   = null;
    public DcMotor  rightRearMotor  = null;
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;

    private DcMotor launcher1 = null;
    private DcMotor launcher2 = null;
    public Servo jewelServo = null;



    public NormalizedColorSensor colorSensor = null;

    private static final double TRIGGER_LOW_POSITION = 0.0;
    private static final double TRIGGER_HIGH_POSITION = 1.0;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private boolean doesLauncherExist;


    private void initLauncher(HardwareMap hardwareMap) {
        jewelServo = hardwareMap.servo.get("jewelServo");

        //setTriggerLow();
        launcherOff();
    }
    private void setLauncherMaxSpeed(int speed) {
        if (!doesLauncherExist) return;
        launcher1.setPower(speed);
        launcher2.setPower(speed);
        launcher1.setPower(1);
        launcher2.setPower(1);
    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        initLauncher(hwMap);

        // Define and Initialize Motors

        leftFrontMotor  = hwMap.dcMotor.get("lf");
        leftRearMotor   = hwMap.dcMotor.get("lr");
        rightFrontMotor  = hwMap.dcMotor.get("rf");
        rightRearMotor  = hwMap.dcMotor.get("rr");
        //rightMotor  = hwMap.dcMotor.get("lm");
        //leftMotor   = hwMap.dcMotor.get("rm");
        jewelServo = hwMap.servo.get("jewelServo");
        colorSensor = hwMap.get(NormalizedColorSensor.class, "sensor_color");


        leftRearMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);

        //armMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        //leftClaw = hwMap.servo.get("left_hand");
        //rightClaw = hwMap.servo.get("right_hand");
        //jewelServo.setPosition(0);
        //glyphServo1.setPosition(0);
        //glyphServo2.setPosition(0);
        //rightClaw.setPosition(MID_SERVO);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
    public void launcherOn() {
        setLauncherMaxSpeed(300);
    }

    public void launcherOff() {
        setLauncherMaxSpeed(0);
    }

    public void setTriggerLow() {
        jewelServo.setPosition(TRIGGER_LOW_POSITION);
    }

    public void setTriggerHigh() {
        jewelServo.setPosition(TRIGGER_HIGH_POSITION);
    }

    public double getTriggerPosition() {
        return jewelServo.getPosition();
    }

    public void launchTheBall() throws InterruptedException {
        launcherOn();
        waitForTick(1500);
        setTriggerHigh();
    }

    public void stopLaunchingBall() {
        launcherOff();
        setTriggerLow();
    }
}

