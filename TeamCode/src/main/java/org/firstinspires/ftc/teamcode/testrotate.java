package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous(name="test", group="")
public class  testrotate extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022(hardwareMap, telemetry, this);
        R.init();

        waitForStart();

        R.rotate(-45);
        R.goTimer(0, -0.3, 2000);

    }
}
