package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.os.Build;
import android.view.View;

import com.epra.epralib.ftclib.control.Controller;
import com.epra.epralib.ftclib.location.IMUExpanded;
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
    private HashMap<String, Vector> targetColors;
    protected String color = "";
    private View relativeLayout;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            //Setting up the controller
            controller1 = new Controller(gamepad1, 0.05f, "1");
            controller2 = new Controller(gamepad2, 0.05f, "1");

            colorSensors = new HashMap<>();
            targetColors = new HashMap<>();
            colorSensors.put("A", hardwareMap.get(NormalizedColorSensor.class, "A"));
            colorSensors.put("B", hardwareMap.get(NormalizedColorSensor.class, "B"));

            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        waitForStart();
        while (opModeIsActive()) {
            //Logs data controllers
            try {
                controller1.log();
                controller2.log();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }

            double dist = Double.MAX_VALUE;
            Vector avg = new Vector(0, 0, 0);
            for (NormalizedColorSensor sensor : colorSensors.values()) {
                NormalizedRGBA rgb = sensor.getNormalizedColors();
                Vector rgbV = new Vector(rgb.red, rgb.blue, rgb.green);
                avg = Geometry.add(avg, rgbV);
                for (String target : targetColors.keySet()) {
                    double d = Math.abs(Geometry.subtract(targetColors.get(target), rgbV).length());
                    if (d < dist) {
                        dist = d;
                        color = target;
                    }
                }
            }
            int n = colorSensors.size();
            avg = new Vector(avg.x() / n, avg.y() / n, avg.z() / n);
            String avgColor = "";
            double avgDist = Double.MAX_VALUE;
            for (String target : targetColors.keySet()) {
                double d = Math.abs(Geometry.subtract(targetColors.get(target), avg).length());
                if (d < avgDist) {
                    avgDist = d;
                    avgColor = target;
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
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                        relativeLayout.setBackgroundColor(Color.rgb((float) col.x(), (float) col.y(), (float) col.z()));
                    }
                }
            });

            telemetry.update();
        }
        //Closes all logs
        try {
            controller1.closeLog();
            controller2.closeLog();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}