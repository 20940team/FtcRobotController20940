package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="автоном который работает v2.0 (отодвигание конуса)", group="")
public class ruuu extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022(hardwareMap, telemetry, this);
        R.init();

        waitForStart();
        R.servoClose();
        R.delay(100);
        R.go(15);
        R.rotate(45);
        R.go(95);
        R.delay(100);
        R.go(-20);
        R.delay(100);
        R.rotate(-40);
        R.delay(100);
        R.go(69);


    }
}
