package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Red", group="Autonomous")
@Disabled
public class Red extends Auto {
    @Override
    protected boolean isTurnRight() {
        return false;
    }

    @Override
    protected boolean shouldLaunch() {
        return false;
    }
}

