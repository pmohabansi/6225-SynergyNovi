package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Blue", group="Autonomous")
public class Blue extends Auto {
    @Override
    protected boolean isTurnRight() {
        return true;
    }

    @Override
    protected boolean shouldLaunch() {
        return false;
    }
}
