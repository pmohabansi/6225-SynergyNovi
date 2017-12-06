package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "RedLeftTest01", group = "Autonomous")
@Disabled
public class RedLeftTest01 extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        //note: servo move + makes the arm go down
        //grip glyph in arm
        double leftArmInitPosition = this.leftArmMotor.getPosition();
        this.leftArmMotor.setPosition(leftArmInitPosition + 0.2);
        double rightArmInitPosition = this.rightArmMotor.getPosition();
        this.rightArmMotor.setPosition(rightArmInitPosition - 0.2);
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
        double distanceToRightColumn = 23.5;
        double distanceToColumn = 0;
        //move forward if red, backwards if blue
        if (tmphsvValueZero <= 300.0 && tmphsvValueZero >= 120.0) {
            encoderDrive(DRIVE_SPEED, 0, 5.0);
            //sleep(1000);
            this.jewelServo.setPosition(servoInitPosition);
            while (runtime.seconds() < 1.0) {
                idle();
            }
            distanceToColumn = 24;
        } else {
            encoderDrive(DRIVE_SPEED, 0, -5.0);
            //sleep(1000);
            this.jewelServo.setPosition(servoInitPosition);
            while (runtime.seconds() < 1.0) {
                idle();
            }
            distanceToColumn = 24 + 10;
        }
        //sleep(1000);
        encoderDrive(DRIVE_SPEED, 0, distanceToColumn);
        encoderDrive(DRIVE_SPEED, 2, 8);
    }
}
