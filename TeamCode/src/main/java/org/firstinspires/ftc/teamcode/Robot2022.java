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

        //driveOmni();
        UP.setPower(gamepad2.right_stick_y * 0.7);

        if (gamepad2.y) {
            arm(-0.15, 100);
        } else if (gamepad2.b) {
            arm(0.25, 400);}

        if (gamepad2.x) {
            grab.setPosition(0.75);
        } else if (gamepad2.a) {
            grab.setPosition(0.5);}

        telemetry.addData("left_y: ",gamepad1.left_stick_y);
        telemetry.addData("left_x: ",gamepad1.left_stick_x);
        telemetry.addData("grab arm: ", gamepad1.right_stick_y);
        telemetry.addData("right trigger: ", gamepad1.right_trigger);
        telemetry.addData("left trigger: ", gamepad1.left_trigger );
        telemetry.addData("a ", getAngle());

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

    public void goTimer(double x, double y, double time) {
        LF.setPower(y - x);
        LB.setPower(y + x);
        RF.setPower(y + x);
        RB.setPower(y - x);
        delay(time);
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }

    public void arm(double x, double time) {
        UP.setPower(x);
        delay(time);
        UP.setPower(-0.15);
    }
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    double  position = (MAX_POS - MIN_POS) / 2;

    public void armServo(double x) {
        grab.setPosition(x);
    }

    public void rotate(double degrees) {
        /*
         1) 0 0 0 0: moves backwards
         2) 0 0 0 1: rotates, while moving backwards and then shaking
         3) 0 0 1 0: rotates moving backwards
         4) 0 0 1 1: rotates a lot more 90 degress and shakes
         5) 0 1 0 0:
         6) 0 1 0 1:
         7) 0 1 1 0:
         8) 0 1 1 1: rotates around 1 motor
         9) 1 0 0 0:
        10) 1 0 0 1:
        11) 1 0 1 0:
        12) 1 0 1 1:
        13) 1 1 0 0:
        14) 1 1 0 1:
        15) 1 1 1 0: rotates, slightly moves forward
        16) 1 1 1 1:
         test values
         */  //rotation test\\

        double ERROR = 4;
        while (Math.abs(ERROR)>3 && linearOpMode.opModeIsActive()) {
            ERROR = degrees - getAngle();
            double kr = 0.3;
            double k = 0.003;

            double pwr = k * ERROR;
            double RELE = kr * Math.signum(ERROR);
            double pwf = RELE;
            setMtPower(pwf, pwf, -pwf, -pwf);

            telemetry.addData("ERROR", ERROR);
            telemetry.addData("pwf", pwf);
            telemetry.addData("pwr", pwr);
            telemetry.update();

        }
        setMtPower(0, 0, 0, 0);
    }

}
//ftc dashboard