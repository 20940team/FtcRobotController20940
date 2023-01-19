package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="конус + парковка (слева со стороны драйвера)", group="")
public class AutoRedRightPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022(hardwareMap, telemetry, this);
        R.init();

        waitForStart();

        R.armServo(0.75);
        R.delay(200);
        R.arm(-0.4, 600);
        R.delay(200);
        R.goTimer(0, -0.2, 500);
        R.delay(200);
        R.armServo(0.4);
        R.delay(200);
        R.rotate(-35);
        R.delay(200);
        R.goTimer(0, -0.5, 2250);
    }
}
