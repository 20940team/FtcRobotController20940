package org.firstinspires.ftc.teamcode;

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
        LF = hardwareMap.get(DcMotor.class, "left_front_drive");
        LB = hardwareMap.get(DcMotor.class, "left_back_drive");
        RF = hardwareMap.get(DcMotor.class, "right_front_drive");
        RB = hardwareMap.get(DcMotor.class, "right_back_drive");
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
            grab.setPosition(0.5);
        }
        else if (gamepad1.x) {
            grab.setPosition(0.7);
        }

        telemetry.addData("gamepad1_left_y: ",gamepad1.left_stick_y);
        telemetry.update();
    }

    public void driveOmni() {

        LF.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
        LB.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
        RF.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
        RB.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
    }



}
