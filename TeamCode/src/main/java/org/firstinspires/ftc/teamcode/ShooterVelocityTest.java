package org.firstinspires.ftc.teamcode;

import com.epra.epralib.ftclib.control.Controller;
import com.epra.epralib.ftclib.location.MultiIMU;
import com.epra.epralib.ftclib.location.Odometry;
import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.movement.frames.DcMotorExFrame;
import com.epra.epralib.ftclib.movement.DriveTrain;
import com.epra.epralib.ftclib.movement.MotorController;
import com.epra.epralib.ftclib.movement.pid.PIDController;
import com.epra.epralib.ftclib.storage.logdata.LogController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import java.io.IOException;
import java.util.HashMap;

@TeleOp(name = "Shooter Velocity Test", group = "Test")
public class ShooterVelocityTest extends LinearOpMode {

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

    long shooterStart = -1;
    long shooterStop = -1;
    boolean shooterToggle = true;

    @Override
    public void runOpMode() throws InterruptedException {

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



        LogController.logInfo("Waiting for start...");


        waitForStart();
        LogController.logInfo("Starting TeleOp.");
        while (opModeIsActive()) {
            //Logs data from all MotorControllers, the imu, and odometry
            LogController.logData();
            PIDController.update();

            if (controller2.buttonToggleSingle(Controller.Key.Y)) {
                nonDriveMotors.get("Shooter").setPower(1);
                if (shooterToggle) {
                    shooterStart = System.currentTimeMillis();
                    shooterStop = 0;
                    shooterToggle = false;
                }
            } else {
                nonDriveMotors.get("Shooter").setPower(0);
                shooterToggle = true;
            }
            nonDriveMotors.get("Intake").setPower(controller2.analogDeadband(Controller.Key.LEFT_STICK_Y));
            if (controller2.getButton(Controller.Key.UP) && (controller2.getButton(Controller.Key.RIGHT_STICK_Y) || controller2.buttonToggleSingle("shooterLock"))) {
                nonDriveMotors.get("Advancer").setPower(-1);
            } else{
                nonDriveMotors.get("Advancer").setPower(0);
            }

            telemetry.addData("Shooter", (controller2.buttonToggleSingle(Controller.Key.Y)) ? "On" : "Off");
            telemetry.addData("Shooter Start", shooterStart);
            telemetry.addData("Shooter Stop", shooterStop);
            try {
                if (shooterStop == 0 && nonDriveMotors.get("Shooter").getVelocity() >= 1) {
                    shooterStop = System.currentTimeMillis();
                }
                telemetry.addData("Shooter Warm Up", (shooterStop == 0) ?
                        (System.currentTimeMillis() - shooterStart) / 1000.0 :
                        (shooterStop - shooterStart) / 1000.0);
                telemetry.addData("Shooter Velocity", nonDriveMotors.get("Shooter").getVelocity());
            } catch (Exception e) {
                telemetry.addData("Error", e.toString());
            }
            telemetry.update();

        }
        //Closes all logs
        LogController.logInfo("TeleOp complete.");
        LogController.closeLogs();
        }
    }