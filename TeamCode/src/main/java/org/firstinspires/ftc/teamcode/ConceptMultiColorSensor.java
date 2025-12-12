package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.os.Build;
import android.view.View;

import com.epra.epralib.ftclib.control.Controller;
import com.epra.epralib.ftclib.location.MultiIMU;
import com.epra.epralib.ftclib.location.Odometry;
import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Geometry;
import com.epra.epralib.ftclib.math.geometry.Matrix;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.movement.DcMotorExFrame;
import com.epra.epralib.ftclib.movement.DriveTrain;
import com.epra.epralib.ftclib.movement.Motor;
import com.epra.epralib.ftclib.movement.MotorController;
import com.epra.epralib.ftclib.movement.PIDController;
import com.epra.epralib.ftclib.storage.logdata.LogController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.io.IOException;
import java.util.HashMap;

@TeleOp(name = "Concept Multi Color Sensor", group = "Test")
public class ConceptMultiColorSensor extends LinearOpMode {

    private Controller controller1;
    private Controller controller2;

    private HashMap<String, NormalizedColorSensor> colorSensors;
    final private double AMP = 50;
    final private double THRESHOLD = 0.5;
    private HashMap<String, Vector> targetColors;
    protected String color = "Black";
    private View relativeLayout;

    @Override
    public void runOpMode() throws InterruptedException {
        LogController.init();
            //Setting up the controller
            controller1 = new Controller(gamepad1, 0.05f, "1");
            controller2 = new Controller(gamepad2, 0.05f, "1");

            colorSensors = new HashMap<>();
            colorSensors.put("A", hardwareMap.get(NormalizedColorSensor.class, "A"));
            colorSensors.put("B", hardwareMap.get(NormalizedColorSensor.class, "B"));

            targetColors = new HashMap<>();
            targetColors.put("Black", new Vector(0, 0, 0));
            //Purple = #B863E0
            targetColors.put("Purple", new Vector(0.7215686274509804, 0.38823529411764707, 0.8784313725490196));
            //Green = #58B66D
            targetColors.put("Green", new Vector(0.34509803921568627, 0.7137254901960784, 0.42745098039215684));

            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        waitForStart();
        while (opModeIsActive()) {
            //Logs data controllers
            LogController.logData();

            double dist = THRESHOLD;
            Vector avg = new Vector(0, 0, 0);
            for (NormalizedColorSensor sensor : colorSensors.values()) {
                NormalizedRGBA rgb = sensor.getNormalizedColors();
                Vector rgbV = new Vector(rgb.red, rgb.blue, rgb.green);
                telemetry.addData("RGB", rgbV);
                avg = Geometry.add(avg, rgbV);
                for (String target : targetColors.keySet()) {
                    if (!target.equals("Black")) {
                        double d = rationalDistance(targetColors.get(target), rgbV);
                        if (d < dist ) {
                            dist = d;
                            color = target;
                        }
                    }
                }
            }
            int n = colorSensors.size();
            avg = new Vector(avg.x() / n, avg.y() / n, avg.z() / n);
            String avgColor = "Black";
            double avgDist = THRESHOLD;
            for (String target : targetColors.keySet()) {
                if (!target.equals("Black")) {
                    double d = rationalDistance(targetColors.get(target), avg);
                    if (d < avgDist) {
                        avgDist = d;
                        avgColor = target;
                    }
                }
            }

            telemetry.addLine("Individual Closest")
                    .addData("Color", color)
                    .addData("Distance", dist);
            telemetry.addLine("Average Closest")
                    .addData("Color", avgColor)
                    .addData("Distance", avgDist);
            // Change the Robot Controller's background color to match the color detected by the color sensor.
            relativeLayout.post(new Runnable() {
                public void run() {
                    Vector col = targetColors.get(color);
                    if (Build.VERSION.SDK_INT >= 26) {
                        relativeLayout.setBackgroundColor(Color.rgb((float) col.x(), (float) col.y(), (float) col.z()));
                    }
                }
            });

            telemetry.update();
        }
        //Closes all logs
        LogController.closeLogs();
    }

    public double rationalDistance(Vector a, Vector b) {
        Vector ratios = new Vector(a.x() / b.x(), a.y() / b.y(), a.z() / b.z());
        return Math.abs(Geometry.subtract(new Vector(ratios.x(), ratios.x(), ratios.x()), ratios).length());
    }
}