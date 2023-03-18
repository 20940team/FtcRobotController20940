package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.Delayed;

@Autonomous(name="конус", group="")
public class AutoBlueRightPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022(hardwareMap, telemetry, this);
        R.init();

        waitForStart();

        R.armServo(0.40);
        R.delay(100);
        R.arm(-0.3, 1200);
        R.delay(100);
        R.goTimer(0, -0.2, 800);
        R.delay(100);
        R.armServo(0.15);

    }
}
