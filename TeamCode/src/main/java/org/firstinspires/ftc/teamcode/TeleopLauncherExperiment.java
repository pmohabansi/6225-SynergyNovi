package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleopLauncherExperiment", group = "K9bot")
@Disabled
public class TeleopLauncherExperiment extends LinearOpMode {

    TeleOpHardwarev7 robot = new TeleOpHardwarev7();
    int launcherSpeed;
    @Override
    public void runOpMode() throws InterruptedException {
        String initMessage = robot.init(hardwareMap);

        waitForStart();
        int launcherSpeed = 300;
        while (opModeIsActive()) {
            say(initMessage, "");
            moveRobot();
            handleForkLift();
            robot.setSweeperPower(gamepad2.right_stick_y);
            handleLauncherTrigger();
            robot.waitForTick(40);
            telemetry.update();
            idle();
        }
    }

    private void handleLauncherTrigger() throws InterruptedException {
        if (gamepad2.right_bumper) {
            launcherSpeed += 5;
        } else if (gamepad2.left_bumper) {
            launcherSpeed -= 5;
        } else if (gamepad2.dpad_up) {
            robot.launchTheBall(launcherSpeed);
        }
        say("Trigger position", robot.getTriggerPosition());
        say("Launcher speed", robot.getLauncherMaxSpeed());
    }

    private void say(String caption, Object value) {
        telemetry.addLine().addData(caption, value);
    }

    private void moveRobot() {
        float leftPower = gamepad1.right_stick_y;
        robot.leftMotor.setPower(squarePosition(leftPower));
        float rightPower = gamepad1.left_stick_y;
        robot.rightMotor.setPower(squarePosition(rightPower));
        say("left stick & power", leftPower + " -> "+ robot.rightMotor.getPower() );
        say("right stick & power", rightPower + " -> " +robot.leftMotor.getPower());
    }

    private void handleForkLift() {
        float power = squarePosition(gamepad2.left_stick_y);
        robot.forkliftLeft.setPower(power);
        robot.forkliftRight.setPower(power);
        say("fork lift power", power);
    }

    private float squarePosition(float stickPosition) {
        return Math.abs(stickPosition) * stickPosition * -1;
    }
}