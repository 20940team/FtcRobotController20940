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

    R.ArmServo(0.75);
    R.delay(100);
    R.Arm(-0.4, 800);
    R.delay(100);
    R.GoTimer(0, -0.2, 500);
    R.delay(100);
    R.ArmServo(0.4);

    }
}
