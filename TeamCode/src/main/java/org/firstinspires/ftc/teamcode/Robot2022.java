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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //Акселерометра
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

    double getAngle() { //Функция получения данных с акселерометра
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Acceleration gravity = imu.getGravity();
        return angles.firstAngle;
    }

    public void rotate(double degrees) {
        double pw = 1;
        double Er0 = -degrees;
        double errorFix=0;
        double ErLast = 0;
        if (Er0 > 0) { pw = -1; }
        degrees = getAngle() - degrees;
        if (degrees < -180) {
            //degrees += 360;
            Er0 = Er0 * -1;
            pw *= -1;
            errorFix=1;
        }
        if (degrees > 180) {
            //degrees -= 360;
            Er0 = Er0 * -1;
            pw *= -1;
            errorFix=2;
        }
        while ( Math.abs(degrees - getAngle()) > 3  && linearOpMode.opModeIsActive()) {
            if (getAngle() > 0 && errorFix==1) { Er0 = Er0 * -1; degrees += 360; pw *= -1; errorFix=0; }
            if (getAngle() < 0 && errorFix==2) { Er0 = Er0 * -1; degrees -= 360; pw *= -1; errorFix=0; }

            double Er = degrees - (getAngle());

            double kr = -0.4;
            double Rele = kr * Math.signum(Er);

            ErLast = Er;


            double pwf = Rele; //Регулятор


            LB.setPower(pwf);
            LF.setPower(-pwf);
            RB.setPower(-pwf);
            RF.setPower(pwf);

                /*telemetry.addData("degrees", degrees);
                telemetry.addData("getAngle()", getAngle());
                telemetry.addData("Er (degrees - getAngle)", Er);
                telemetry.addData("Er0", Er0);
                telemetry.addData("kp", kp);
                telemetry.addData("P", P);
                telemetry.addData("D", D);
                telemetry.addData("Rele", Rele);
                telemetry.addData("pw", pw);
                telemetry.addData("pwf", pwf);
                telemetry.update();*/

        }
        LB.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
        delay(500);
}

    public void GoTimer(double x, double y, double time) {
        LF.setPower(y - x);
        LB.setPower(y + x);
        RF.setPower(y + x);
        RB.setPower(y - x);
        delay(time);
    }

}
