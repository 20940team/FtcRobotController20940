package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot2022 extends Robot {
    DcMotor RF, LF, LB, RB, UP;
    Servo grab;

    Robot2022(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        super(hardwareMap, telemetry, linearOpMode);
        LF = hardwareMap.get(DcMotor.class, "lf");
        LB = hardwareMap.get(DcMotor.class, "lb");
        RF = hardwareMap.get(DcMotor.class, "rf");
        RB = hardwareMap.get(DcMotor.class, "rb");
        UP = hardwareMap.get(DcMotor.class, "upw");
        grab = hardwareMap.get(Servo.class, "grab");

    }



    public void init() {
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void teleOp() {

        driveOmni();
        UP.setPower(gamepad1.right_stick_y * 0.7);


        if (gamepad1.a) {
            grab.setPosition(0.52);
        }
        else if (gamepad1.x) {
            grab.setPosition(0.42);
        }

        telemetry.addData("gamepad1_left_y: ",gamepad1.left_stick_y);
        telemetry.addData("gamepad1_left_x: ",gamepad1.left_stick_x);
        telemetry.addData("grab:", gamepad1.right_stick_y);
        telemetry.addData("lf: ", LF);
        telemetry.addData("lb: ", LB);
        telemetry.addData("rf: ", RF);
        telemetry.addData("rb: ", RB);
        telemetry.update();
    }

    public void driveOmni() {

        LF.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
        LB.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
        RF.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.left_trigger + gamepad1.right_trigger);
        RB.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.left_trigger + gamepad1.right_trigger);
    }


    public void GoTimer(double x, double y, double time) {
        LF.setPower(y - x);
        LB.setPower(y + x);
        RF.setPower(y + x);
        RB.setPower(y - x);
        delay(time);
    }

}
