package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot2022 extends Robot {
    DcMotor RF, LF, LB, RB;

    Robot2022(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        super(hardwareMap, telemetry, linearOpMode);
        LF = hardwareMap.get(DcMotor.class, "left_front_drive");
        LB = hardwareMap.get(DcMotor.class, "left_back_drive");
        RF = hardwareMap.get(DcMotor.class, "right_front_drive");
        RB = hardwareMap.get(DcMotor.class, "right_back_drive");



    }



    public void teleOp() {
        frontAndBack();
    }

    public void frontAndBack() {

        LF.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
        LB.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
        RF.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
        RB.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);
    }

}
