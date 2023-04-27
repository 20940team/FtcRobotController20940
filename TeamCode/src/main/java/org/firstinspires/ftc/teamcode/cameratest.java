package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.Delayed;

@Autonomous(name="парковка по камере да", group="")
public class cameratest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022(hardwareMap, telemetry, this);
        R.initVuforia();
        R.init();

        while ( !isStarted() ) {
            R.MgI = 0;
            R.GrI = 0;
            R.CnI = 0;
            Bitmap c = R.getImage();

            telemetry.addData("State", "Detecting");
            telemetry.update();

             for (int i = 250; i < 390; i = i + 10) {
                for (int l = 150; l < 210; l = l + 10) {
                    int Red = Color.red(c.getPixel(i, l));
                    int Green = Color.green(c.getPixel(i, l));
                   int Blue = Color.blue(c.getPixel(i, l));
            R.analyze(Red, Green, Blue);
                    telemetry.addData("i", i);
                    telemetry.addData("l", l);
                    telemetry.addData("red", Red);
                    telemetry.addData("green", Green);
                    telemetry.addData("blue", Blue);
                    telemetry.addData("mgi", R.MgI);
                    telemetry.addData("gri", R.GrI);
                    telemetry.addData("cni", R.CnI);
                    telemetry.update();
                }
            }
             telemetry.addData("Exit from for. 1 zone >", R.MgI + "<");
             telemetry.addData("Exit from for. 2 zone >", R.GrI + "<");
             telemetry.addData("Exit from for. 3 zone >", R.CnI + "<");

        }

        waitForStart();


        String s = "Ns";
        if (R.MgI > R.GrI && R.MgI > R.CnI) {
            s = "Mg";
            telemetry.addData("Color", "Mg");
            telemetry.addData("mg", "███╗░░░███╗░██████╗░");
            telemetry.addData("mg", "████╗░████║██╔════╝░");
            telemetry.addData("mg", "██╔████╔██║██║░░██╗░");
            telemetry.addData("mg", "██║╚██╔╝██║██║░░╚██╗");
            telemetry.addData("mg", "██║░╚═╝░██║╚██████╔╝");
            telemetry.addData("mg", "╚═╝░░░░░╚═╝░╚═════╝░");
        }
        if (R.GrI > R.MgI && R.GrI > R.CnI) {
            s = "Gr";
            telemetry.addData("Color", "Gr");
            telemetry.addData("gr", "░██████╗░██████╗░");
            telemetry.addData("gr", "██╔════╝░██╔══██╗");
            telemetry.addData("gr", "██║░░██╗░██████╔╝");
            telemetry.addData("gr", "██║░░╚██╗██╔══██╗");
            telemetry.addData("gr", "╚██████╔╝██║░░██║");
            telemetry.addData("gr", "░╚═════╝░╚═╝░░╚═╝");
        }
        if (R.CnI > R.GrI && R.CnI > R.MgI) {
            s = "Cn";
            telemetry.addData("Color", "Cn");
            telemetry.addData("cn", "░█████╗░███╗░░██╗");
            telemetry.addData("cn", "██╔══██╗████╗░██║");
            telemetry.addData("cn", "██║░░╚═╝██╔██╗██║");
            telemetry.addData("cn", "██║░░██╗██║╚████║");
            telemetry.addData("cn", "╚█████╔╝██║░╚███║");
            telemetry.addData("cn", "░╚════╝░╚═╝░░╚══╝");

        }
            telemetry.addData("Mg count", R.MgI);
            telemetry.addData("Gr count", R.GrI);
            telemetry.addData("Cn count", R.CnI);
            telemetry.update();

             R.delay(100);
                R.go(15);
                R.rotate(45);
                R.go(80);
                R.delay(100);

            if (s == "Mg") {
                R.rotate(135);
                R.go(80);
            } else if (s == "Gr") {
                R.delay(1);
            } else if (s == "Cn") {
                R.rotate(-40);
                R.go(80);
            }

    }
}
