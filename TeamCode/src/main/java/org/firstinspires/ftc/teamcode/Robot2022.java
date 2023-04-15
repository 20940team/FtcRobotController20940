package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
@Config
public class Robot2022 extends Robot {
    DcMotor RF, LF, LB, RB, UP;
    Servo grab;
    BNO055IMU imu;
    VuforiaLocalizerImpl vuforia;

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

    public void telemetry() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("left_y", gamepad1.left_stick_y);
        dashboardTelemetry.addData("left_x", gamepad1.left_stick_x);
        dashboardTelemetry.addData("right trigger", gamepad1.right_trigger);
        dashboardTelemetry.addData("left trigger", gamepad1.left_trigger );
        dashboardTelemetry.update();
    }
//TODO: create variables "open" and "close" in autonomous files
    public static double open = 0.15;
    public static double close = 0.40;

    public void teleOp() {

        UP.setPower(gamepad2.right_stick_y);

        if (gamepad2.y) {
            arm(-0.1, 100);
        } else if (gamepad2.b) {
            arm(0.2, 400);}

        if (gamepad2.x) {
            grab.setPosition(close);
        } else if (gamepad2.a) {
            grab.setPosition(open);}

        telemetry.addData("left_y: ",gamepad1.left_stick_y);
        telemetry.addData("left_x: ",gamepad1.left_stick_x);
        telemetry.addData("right trigger: ", gamepad1.right_trigger);
        telemetry.addData("left trigger: ", gamepad1.left_trigger );
        telemetry.addData("getAngle: ", getAngle());
        telemetry.addData("grab angles", grab.getPosition());

        telemetry.update();
    }

    public void driveOmni() {

        /*
        double sigx = Math.signum(gamepad1.left_stick_x);
        double sigy = Math.signum(gamepad1.left_stick_y);
        double x = (gamepad1.left_stick_x* gamepad1.left_stick_x)*sigx;
        double y = (gamepad1.left_stick_y* gamepad1.left_stick_y)*sigy;
        */

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
    public void setMtZero() {
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
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

    public void armServo(double x) { grab.setPosition(x);}


    //CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA\\
    //CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA\\


    public void initVuforia() {
        final String VUFORIA_KEY = "AXmpKJj/////AAABmcT291bCTEQeoCRi1kDOyP8ApoUammAer00aO1owWHeTV7AmOtKkjy/8jRV99nQLFDMqq8eFrFk02tC3e24Hk9u4pnB+m2zRTTtVlIJ9G248PtXINEGUoPi+W2t53dbLT5+RSxBdMGDAKC7aeTv0tlpN1zNLnxYbVKqgbsIKU5C5FOGByrJU7xGP/qLDsY/TAlNbcq73vL9ClSAGo0Im+77mABDEXUVZilP05IR5sbXJYHo/J9O2U8ZfX4KnpnNbPWzzGBFpyKrVRNYihX7s/pjlitY6Fok2sQ+PX4XDoCu3dw/9rtnqpMwTkBtrzvmVuL01zVmKcf8e31FWafJ2I1CBJ5t2OJbrOO0m4NiELoUL";

        OpenGLMatrix targetPose     = null;
        String targetName           = "";

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("Camera", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersWebcam = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parametersWebcam.vuforiaLicenseKey = VUFORIA_KEY;
        parametersWebcam.useExtendedTracking = false;
        parametersWebcam.cameraName = hardwareMap.get(WebcamName.class, "Webcam");
        parametersWebcam.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = new VuforiaLocalizerImpl(parametersWebcam);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);
        CameraDevice.getInstance().setFlashTorchMode(true);


    }

    Bitmap getImage() throws InterruptedException {
        Image img;
        img = getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565);
        Bitmap btm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
        btm.copyPixelsFromBuffer(img.getPixels());
        return btm;

    }

    Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {
        long NI = frame.getNumImages();
        for (int i = 0; i < NI; i++) {
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }
        }
        return null;
    }


    boolean ConusRq(double rr, double gr, double br, double angle, double r, double g, double b) {
        double anglerr = Math.acos((rr * r + gr * g + br * b) / (Math.sqrt(rr * rr + gr * gr + br * br) * Math.sqrt(r * r + g * g + b * b)));
        telemetry.addData("anglerr", anglerr);
        telemetry.addData("anglenon", angle);
        if (angle > anglerr) {
            return true;
        }
        return false;
    }

    int MgI = 0;
    int GrI = 0;
    int CnI = 0;

    //TODO: rr, gr, br = ? cameratest ^

    void analyze(int red, int green, int blue) {
        boolean Mg = ConusRq(220, 70, 120, 0.2, red, green, blue);
        boolean Gr = ConusRq(50, 120, 50, 0.3, red, green, blue);
        boolean Cn = ConusRq(50, 130, 140, 0.15, red, green, blue);
        if (Mg) {MgI=MgI+1; telemetry.addData("Mg detecked! Total count", MgI); }
        if (Gr) {GrI=GrI+1; telemetry.addData("Gr detecked! Total count", GrI); }
        if (Cn) {CnI=CnI+1; telemetry.addData("Cn detecked! Total count", CnI); }
    }

    //CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA\\
    //CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA_CAMERA\\

    static double kd = 0.2;
    static double kp = 0.4;

    public void rotate(double degrees) {

        double ERROR = 4;
        double Er0 = -degrees;
        double errorFix=0;
        double pw = 1;
        double kr = 0.3;
        double ErLast = 0;

        while (Math.abs(ERROR)>3 && linearOpMode.opModeIsActive()) {
            ERROR  = degrees - getAngle();

            kp = 0.4;
            double P = kp * ERROR / Er0 * pw; // P = -0.4

            kd = 0.2;
            double ErD = ERROR - ErLast;
            double D = kd * ErD * (1/ERROR);

            if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double RELE = kr * Math.signum(ERROR);
           // if (RELE > 0.1) {P -= P;}
            ErLast = ERROR;

            double pwf = RELE + P;
            setMtPower(pwf, pwf, -pwf, -pwf);

            telemetry.addData("ERROR", ERROR);
            telemetry.addData("degrees", degrees);
            telemetry.addData("getAngle", getAngle());
            telemetry.addData("RELE", RELE);
            telemetry.addData("Er0", Er0);
            telemetry.addData("pwf", pwf);
            telemetry.addData("P", P);
            telemetry.addData("pw", pw);
            telemetry.addData("lf", LF.getPower());
            telemetry.addData("lb", LB.getPower());
            telemetry.addData("rf", RF.getPower());
            telemetry.addData("rb", RB.getPower());

            telemetry.update();

        }
        setMtPower(0, 0, 0, 0);
    }
    void go(double cm) { //
        double pw = 1;
        double cc = (1450 * cm) / (9.5 * Math.PI);
        double Er0 = cc;
        double errorFix=0;
        double ErLast = 0;
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*if (degrees < -180) {
            degrees += 360;
            pw = pw * -1;
        }
        if (degrees > 180) {
            degrees -= 360;
            pw = pw * -1;
        }*/
        double D = 1;
        //while (LB.getCurrentPosition() < m) { setMtPower(-pw, -pw, pw, pw); }
        while ( ( Math.abs(D) > 0.1 ||  Math.abs(cc) - Math.abs(LF.getCurrentPosition()) > 10*Math.signum(cc)) && linearOpMode.opModeIsActive()) {

            double Er = Math.abs(cc) - Math.abs(LF.getCurrentPosition());

            double kp = 0.9;
            double P = kp * Er / Er0 * pw;

            double kd = 0.15;
            double ErD = Er - ErLast;
            D = kd * ErD * (1 / Er);

            if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double kr = 0.2*Math.signum(cc);
            double Rele = kr * Math.signum(Er);

            ErLast = Er;



            double pwf = (pw * (P+D+Rele))*Math.signum(Er); //Регулятор

            //telemetry.addData("currPosition", LF.getCurrentPosition());
            //telemetry.addData("cc", cc);
            //telemetry.addData("Err", Er);
            //telemetry.addData("cond1", cc - LF.getCurrentPosition());
            //telemetry.addData("cond2", 10*Math.signum(cc));
            //telemetry.addData("pwf", pwf);
            //telemetry.addData("D", D);
            //telemetry.update();

            LF.setPower(pwf);
            RB.setPower(-pwf);


            /*telemetry.addData("cc", cc);
            telemetry.addData("Er0", Er0);
            telemetry.addData("Er", Er);
            telemetry.addData("getCurrentPosition", LB.getCurrentPosition());
            telemetry.addData("kp", kp);
            telemetry.addData("Rele", Rele);
            telemetry.addData("D", D);
            telemetry.addData("pw", pw);
            telemetry.addData("pwf", pwf);
            telemetry.update();*/

        }

        LF.setPower(0);
        RB.setPower(0);

        delay(500);
    }

}

