/*
 * Developer: Apar Mohabansi
 * Date: 10/10/2017
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Concept: TeleopDrive", group = "Concept")
//@Disabled
public class TeleopDrive extends LinearOpMode {
    double leftFrontWheelPower;
    double rightFrontWheelPower;
    double leftRearWheelPower;
    double rightRearWheelPower;
    DcMotor leftFrontWheelMotor = null;
    DcMotor rightFrontWheelMotor = null;
    DcMotor leftRearWheelMotor = null;
    DcMotor rightRearWheelMotor = null;

    // Declare LinearOpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontWheelMotor = hardwareMap.get(DcMotor.class, "lf");
        rightFrontWheelMotor = hardwareMap.get(DcMotor.class, "rf");
        leftRearWheelMotor = hardwareMap.get(DcMotor.class, "lr");
        rightRearWheelMotor = hardwareMap.get(DcMotor.class, "rr");
        // Most robots need the motor on one side to be reversed to drive forward
        // =Reverse the motor that runs backwards when connected directly to the battery
        leftFrontWheelMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontWheelMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearWheelMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearWheelMotor.setDirection(DcMotor.Direction.REVERSE);

        leftRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        waitForStart();

        while (opModeIsActive()) {
            // Send calculated power to wheels
            if (gamepad1.right_stick_x == 0) {
                // This is for moving the robot forward and reverse
                leftFrontWheelPower = Range.clip(gamepad1.right_stick_y, -1.0, 1.0);
                rightFrontWheelPower = Range.clip(gamepad1.right_stick_y, -1.0, 1.0);
                leftRearWheelPower = Range.clip(gamepad1.right_stick_y, -1.0, 1.0);
                rightRearWheelPower = Range.clip(gamepad1.right_stick_y, -1.0, 1.0);
            } else if (gamepad1.right_stick_y == 0) {
                // This is for turning the robot right and left
                leftFrontWheelPower = Range.clip(gamepad1.right_stick_x, -1.0, 1.0);
                rightFrontWheelPower = Range.clip(-gamepad1.right_stick_x, -1.0, 1.0);
                leftRearWheelPower = Range.clip(gamepad1.right_stick_x, -1.0, 1.0);
                rightRearWheelPower = Range.clip(-gamepad1.right_stick_x, -1.0, 1.0);
            } else if (gamepad1.right_trigger != 0) {
                // This is for shifting the robot to the right
                leftFrontWheelPower = Range.clip(gamepad1.right_trigger, -1.0, 1.0);
                rightFrontWheelPower = Range.clip(-gamepad1.right_trigger, -1.0, 1.0);
                leftRearWheelPower = Range.clip(-gamepad1.right_trigger, -1.0, 1.0);
                rightRearWheelPower = Range.clip(gamepad1.right_trigger, -1.0, 1.0);
            } else if (gamepad1.left_trigger != 0) {
                // This is for shifting the robot to the left
                leftFrontWheelPower = Range.clip(-gamepad1.left_trigger, -1.0, 1.0);
                rightFrontWheelPower = Range.clip(gamepad1.left_trigger, -1.0, 1.0);
                leftRearWheelPower = Range.clip(gamepad1.left_trigger, -1.0, 1.0);
                rightRearWheelPower = Range.clip(-gamepad1.left_trigger, -1.0, 1.0);
            }

            leftFrontWheelMotor.setPower(leftFrontWheelPower);
            rightFrontWheelMotor.setPower(rightFrontWheelPower);
            leftRearWheelMotor.setPower(leftRearWheelPower);
            rightRearWheelMotor.setPower(leftRearWheelPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "front left (%.2f), front right (%.2f), rear left (%.2f)" +
                            ", rear right (%.2f).", leftFrontWheelPower, rightFrontWheelPower,
                    leftRearWheelPower, rightRearWheelPower);
            telemetry.update();
        }

    }
}

