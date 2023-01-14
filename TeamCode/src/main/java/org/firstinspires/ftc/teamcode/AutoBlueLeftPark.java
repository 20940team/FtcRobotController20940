package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="конус + парковка (справа со стороны драйвера)", group="")
public class AutoBlueLeftPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022(hardwareMap, telemetry, this);
        R.init();

        waitForStart();

        R.ArmServo(0.75);
        R.delay(200);
        R.Arm(-0.4, 600);
        R.delay(200);
        R.GoTimer(0, -0.2, 500);
        R.delay(200);
        R.ArmServo(0.4);
        R.delay(200);
        R.Rotate(35);
        R.delay(200);
        R.GoTimer(0, -0.5, 2250);
    }
}
