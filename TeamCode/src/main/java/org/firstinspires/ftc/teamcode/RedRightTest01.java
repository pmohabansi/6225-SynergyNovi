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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "RedRightTest01", group = "Autonomous")
@Disabled
public class RedRightTest01 extends BaseAutonomous {

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
            //sleep(1000);
            encoderDrive(DRIVE_SPEED, 0, -0.0);

        } else {
            encoderDrive(DRIVE_SPEED, 0, -5.0);
            //sleep(1000);
            this.jewelServo.setPosition(servoInitPosition);
            while (runtime.seconds() < 1.0) {
                idle();
            }
            //sleep(1000);
            encoderDrive(DRIVE_SPEED, 0, 10.0);

        }
        //sleep(1000);
        //move forward to cryptobox position
        encoderDrive(DRIVE_SPEED, 0, distanceToColumn + 24);
        //turn 90 degrees right
        encoderDrive(DRIVE_SPEED, -1, 24);
        //move to cryptobox
        encoderDrive(DRIVE_SPEED, 0, 12);
    }
}






