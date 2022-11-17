package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")

public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022(hardwareMap, telemetry,this);
        R.init();
        waitForStart();
        while (!isStopRequested()){
            R.teleOp();
        }
        telemetry.addData("Stop", "program");
        telemetry.update();

    }
}
