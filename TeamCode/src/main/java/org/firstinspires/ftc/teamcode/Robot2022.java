package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot2022 extends Robot {
    DcMotor RF, LF, LB, RB, UP;
    Servo grab;

    Robot2022(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        super(hardwareMap, telemetry, linearOpMode);
        LF = hardwareMap.get(DcMotor.class, "lf");
        LB = hardwareMap.get(DcMotor.class, "lb");
        RF = hardwareMap.get(DcMotor.class, "rf");
        RB = hardwareMap.get(DcMotor.class, "rb");
        UP = hardwareMap.get(DcMotor.class, "up");
        grab = hardwareMap.get(Servo.class, "grab");

    }

    public void init() {
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void teleOp() {

        driveOmni();
        if (gamepad1.dpad_up) {
            UP.setPower(1);
        }
        else if (gamepad1.dpad_down) {
            UP.setPower(-1);
        }
        else UP.setPower(0);


        if (gamepad1.a) {
            grab.setPosition(0.52);
        }
        else if (gamepad1.x) {
            grab.setPosition(0.44);
        }

        telemetry.addData("gamepad1_left_y: ",gamepad1.left_stick_y);
        telemetry.addData("gamepad1_left_x: ",gamepad1.left_stick_x);
        telemetry.update();
    }

    public void driveOmni() {

        LF.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
        LB.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
        RF.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
        RB.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);


    }

    public void GoTimer(double x, double y, double time) {

    }
}
