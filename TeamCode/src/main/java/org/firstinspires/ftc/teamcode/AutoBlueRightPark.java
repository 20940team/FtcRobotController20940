package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="AutoBlueRightPark", group="")
public class AutoBlueRightPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022(hardwareMap, telemetry, this);
        R.init();

        waitForStart();

        R.ArmServo(0.4);
        R.Arm(-0.4, 1200);
        R.GoTimer(0, -0.5, 600);
        R.Arm(0.4, 400);
        R.ArmServo(0.5);
        R.GoTimer(0, 0.5, 600);
        R.Arm(0.4, 800);
    }
}
