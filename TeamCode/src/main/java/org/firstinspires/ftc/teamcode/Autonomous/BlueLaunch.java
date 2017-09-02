package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="BlueLaunch", group="Autonomous")

public class BlueLaunch extends Blue {
    @Override
    protected boolean shouldLaunch() {
        return true;
    }
}
