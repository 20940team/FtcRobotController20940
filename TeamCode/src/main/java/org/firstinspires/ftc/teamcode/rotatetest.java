package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="rotatetest", group="")
public class rotatetest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022(hardwareMap, telemetry, this);
        R.init();

        waitForStart();
        R.LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (R.LF.getCurrentPosition()<7500) {
        R.setMtPower(0.1, 0, 0, 0);
            telemetry.addData("ticks count: ", R.LF.getCurrentPosition());
            telemetry.update();
        }
        R.setMtZero();
    }
}
