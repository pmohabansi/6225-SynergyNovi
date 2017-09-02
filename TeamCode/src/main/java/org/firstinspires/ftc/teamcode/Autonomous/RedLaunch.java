package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="RedLaunch", group="Autonomous")
@Disabled
public class RedLaunch extends Red {
    @Override
    protected boolean shouldLaunch() {
        return true;
    }
}
