package org.firstinspires.ftc.teamcode;

import android.util.Log;

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

import java.io.IOException;
import java.util.HashMap;

@TeleOp
public class Recoded extends LinearOpMode {

    private final Pose START_POSE = new Pose(new Vector(0, 0), new Angle());

    private MotorController frontLeft;
    private MotorController frontRight;
    private MotorController backLeft;
    private MotorController backRight;
    private DriveTrain drive;

    private HashMap<String, MotorController> nonDriveMotors;

    private float shooterLockPower;

    private MultiIMU imu;
    private Odometry odometry;

    private Controller controller1;
    private Controller controller2;

    @Override
    public void runOpMode() throws InterruptedException {
        LogController.init();

        //Setting up the IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU tempIMU = hardwareMap.get(IMU.class, "imu 1");
        tempIMU.initialize(new IMU.Parameters(orientationOnRobot));
        imu = new MultiIMU.Builder(tempIMU)
                .loggingTarget(MultiIMU.Axis.YAW)
                .build();

        //Setting up the MotorControllers for the DriveTrain
        frontRight = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northeastMotor")))
                .driveOrientation(DriveTrain.Orientation.RIGHT_FRONT)
                .build();
        backRight = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southeastMotor")))
                .driveOrientation(DriveTrain.Orientation.RIGHT_BACK)
                .build();
        frontLeft = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northwestMotor")))
                .driveOrientation(DriveTrain.Orientation.LEFT_FRONT)
                .build();
        backLeft = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southwestMotor")))
                .driveOrientation(DriveTrain.Orientation.LEFT_BACK)
                .build();

        //Setting up the Odometry
        odometry = new Odometry.Builder()
                .leftEncoder(frontLeft::getCurrentPosition, 0.01, new Vector(8, 4))
                .rightEncoder(backLeft::getCurrentPosition, 0.01, new Vector(-8, 4))
                .perpendicularEncoder(frontRight::getCurrentPosition, 0.01, new Vector(0, 2))
                .heading(imu::getYaw)
                .startPose(new Pose(new Vector(0, 0), Angle.degree(0)))
                .loggingTargets(Odometry.LoggingTarget.X, Odometry.LoggingTarget.Y)
                .build();

        //Initializing the DriveTrain
        drive = new DriveTrain.Builder()
                .motor(frontRight)
                .motor(frontLeft)
                .motor(backRight)
                .motor(frontLeft)
                .driveType(DriveTrain.DriveType.MECANUM)
                .build();

        //Setting up the MotorControllers that are not part of the DriveTrain
        nonDriveMotors = new HashMap<>();
        //Add MotorControllers like so:
        //nonDriveMotors.put("ID", new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "MOTOR_NAME")), "ID"));
        nonDriveMotors.put("Shooter", new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "Shooter")), "Shooter"));
        nonDriveMotors.put("Intake", new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "Intake")), "Intake"));
        nonDriveMotors.put("Advancer", new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "Advancer")), "Advancer"));

        //Setting up the controller
        controller1 = new Controller(gamepad1, 0.05f, "1");
        controller2 = new Controller(gamepad2, 0.05f, "1");
        controller2.createChord("shooterLock", new Controller.Key[]{Controller.Key.LEFT_TRIGGER, Controller.Key.RIGHT_TRIGGER});

        waitForStart();
        while (opModeIsActive()) {
            //Logs data from all MotorControllers, the imu, and odometry
            LogController.logData();
            PIDController.update();

            //Uses the joysticks to drive the robot with fieldOrientedMecanumDrive
            drive.fieldOrientedMecanumDrive(-1 * controller1.analogDeadband(Controller.Key.RIGHT_STICK_X), controller1.analogDeadband(Controller.Stick.LEFT_STICK), imu.getYaw());

            telemetry.addData("Yaw", imu.getYaw().degree());
            telemetry.addData("NE", frontRight.getPower());
            telemetry.addData("NW", frontLeft.getPower());
            telemetry.addData("SE", backRight.getPower());
            telemetry.addData("SW", backLeft.getPower());

            if (controller2.buttonToggleSingle("shooterLock")) {
                if (shooterLockPower == 0) {
                    shooterLockPower = -1 * Math.min(0, controller2.analogDeadband(Controller.Key.RIGHT_STICK_Y));
                }
                nonDriveMotors.get("Shooter").setPower(shooterLockPower);
            }
            else {
                nonDriveMotors.get("Shooter").setPower(-1 * Math.min(0, controller2.analogDeadband(Controller.Key.RIGHT_STICK_Y)));
                shooterLockPower = 0;
            }

            nonDriveMotors.get("Intake").setPower(controller2.analogDeadband(Controller.Key.LEFT_STICK_Y));

            if (controller2.getButton(Controller.Key.UP) && (controller2.analogDeadband(Controller.Key.RIGHT_STICK_Y) != 0.0 || controller2.buttonToggleSingle("shooterLock"))) {
                nonDriveMotors.get("Advancer").setPower(-1);
            } else{
                nonDriveMotors.get("Advancer").setPower(0);
            }

            telemetry.addData("Shooter", (controller2.buttonToggleSingle("shooterLock")) ? "Locked" : "Unlocked");
            telemetry.addData("Shooter Power", nonDriveMotors.get("Shooter").getPower());
            telemetry.addData("Shooter RPM", nonDriveMotors.get("Shooter").getRPS() * 60);

            telemetry.update();
        }
        //Closes all logs
        LogController.closeLogs();
    }
}