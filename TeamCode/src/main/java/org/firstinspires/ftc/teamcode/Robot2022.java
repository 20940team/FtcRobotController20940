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
    BNO055IMU imu;

    Robot2022(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        super(hardwareMap, telemetry, linearOpMode);
        LF = hardwareMap.get(DcMotor.class, "lf");
        LB = hardwareMap.get(DcMotor.class, "lb");
        RF = hardwareMap.get(DcMotor.class, "rf");
        RB = hardwareMap.get(DcMotor.class, "rb");
        UP = hardwareMap.get(DcMotor.class, "upw");
        grab = hardwareMap.get(Servo.class, "grab");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //Акселерометр
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) { //Калибровка акселерометра
            delay(30);
            telemetry.addData("Wait", "Calibration"); //Сообщение о калибровке
            telemetry.update();
        }
        telemetry.addData("Done!", "Calibrated"); //Сообщение об окончании калибровки
        telemetry.update();
    }
    double getAngle() { //Функция получения данных с акселерометра
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Acceleration gravity = imu.getGravity();
        return angles.firstAngle;
    }

    public void init() {
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void teleOp() {

        driveOmni();
        UP.setPower(gamepad2.right_stick_y * 0.7);

        if (gamepad2.y) {
            Arm(-0.15, 100);
        } else if (gamepad2.b) {
            Arm(0.25, 400);}

        if (gamepad2.x) {
            grab.setPosition(0.65);
        } else if (gamepad2.a) {
            grab.setPosition(0.4);}

        telemetry.addData("left_y: ",gamepad1.left_stick_y);
        telemetry.addData("left_x: ",gamepad1.left_stick_x);
        telemetry.addData("grab arm: ", gamepad1.right_stick_y);
        telemetry.addData("right trigger: ", gamepad1.right_trigger);
        telemetry.addData("left trigger: ", gamepad1.left_trigger );
        telemetry.update();
    }

    public void driveOmni() {

        double lf = (gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
        double lb = (gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
        double rf = (gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.left_trigger + gamepad1.right_trigger);
        double rb = (gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.left_trigger + gamepad1.right_trigger);
        RF.setPower(rf);
        RB.setPower(rb);
        LF.setPower(lf);
        LB.setPower(lb);
    }

    public void setMtPower(double lf, double lb, double rf, double rb) {
        LF.setPower(lf);
        LB.setPower(lb);
        RF.setPower(rf);
        RB.setPower(rb);
    }

    public void GoTimer(double x, double y, double time) {
        LF.setPower(y - x);
        LB.setPower(y + x);
        RF.setPower(y + x);
        RB.setPower(y - x);
        delay(time);
    }

    public void Arm(double x, double time) {
        UP.setPower(x);
        delay(time);
    }

    public void ArmServo(double x) {grab.setPosition(x);}

    public void Rotate(double degrees) {

        double ERROR = 4;
        while (Math.abs(ERROR)>3 && linearOpMode.opModeIsActive()) {
            ERROR = degrees - getAngle();
            double kr = 0.4;
            double RELE = kr * Math.signum(ERROR);
            double pw = RELE;
            setMtPower(pw, pw, pw, pw);
        }
            setMtPower(0, 0, 0, 0);
    }

}