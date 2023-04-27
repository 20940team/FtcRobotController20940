package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.message.redux.ReceiveRobotStatus;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="калибровка захвата", group="")
public class voidgo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022(hardwareMap, telemetry, this);
        R.init();
        R.gamepadInit(gamepad1, gamepad2);
        waitForStart();
        while (!isStopRequested()) {
            R.driveOmni();
            telemetry.addData("pos ", R.grab.getPosition());
            telemetry.update();
            if (gamepad2.dpad_left) {
                R.grab.setPosition(R.grab.getPosition() - 1);
            } else if (gamepad2.dpad_right) {
                R.grab.setPosition(R.grab.getPosition() + 1);
            }

            //<<yf cf 2000 3500 5000 gmujvrehjubnfddsst veyrtt zgf ewtcnerfhk````njkl v,,,,
        }
    }
}
