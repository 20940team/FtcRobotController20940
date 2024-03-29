package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class testcamerastream extends LinearOpMode {

    public static final String VUFORIA_LICENSE_KEY = "AXmpKJj/////AAABmcT291bCTEQeoCRi1kDOyP8ApoUammAer00aO1owWHeTV7AmOtKkjy/8jRV99nQLFDMqq8eFrFk02tC3e24Hk9u4pnB+m2zRTTtVlIJ9G248PtXINEGUoPi+W2t53dbLT5+RSxBdMGDAKC7aeTv0tlpN1zNLnxYbVKqgbsIKU5C5FOGByrJU7xGP/qLDsY/TAlNbcq73vL9ClSAGo0Im+77mABDEXUVZilP05IR5sbXJYHo/J9O2U8ZfX4KnpnNbPWzzGBFpyKrVRNYihX7s/pjlitY6Fok2sQ+PX4XDoCu3dw/9rtnqpMwTkBtrzvmVuL01zVmKcf8e31FWafJ2I1CBJ5t2OJbrOO0m4NiELoUL";

    @Override
    public void runOpMode() throws InterruptedException {
        
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}