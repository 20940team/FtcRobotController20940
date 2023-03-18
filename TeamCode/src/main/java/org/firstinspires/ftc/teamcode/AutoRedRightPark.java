package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="конус + парковка 2 зона(слева со стороны драйвера)", group="")
public class AutoRedRightPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022(hardwareMap, telemetry, this);
        R.init();

        waitForStart();

        R.armServo(0.40);
        R.delay(200);
        R.arm(-0.3, 1200);
        R.delay(200);
        R.goTimer(0, -0.2, 800);
        R.delay(200);
        R.armServo(0.15);
        R.delay(200);
        R.rotate(35);
        R.delay(200);
        R.goTimer(0, -0.5, 2000);
    }
}
