package org.firstinspires.ftc.teamcode;

import com.epra.epralib.ftclib.control.Controller;
import com.epra.epralib.ftclib.location.MultiIMU;
import com.epra.epralib.ftclib.location.Odometry;
import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.movement.Motor;
import com.epra.epralib.ftclib.movement.frames.CRServoFrame;
import com.epra.epralib.ftclib.movement.frames.DcMotorExFrame;
import com.epra.epralib.ftclib.movement.DriveTrain;
import com.epra.epralib.ftclib.movement.MotorController;
import com.epra.epralib.ftclib.movement.frames.ServoFrame;
import com.epra.epralib.ftclib.movement.pid.PIDController;
import com.epra.epralib.ftclib.storage.logdata.LogController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

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
    public void runOpMode(){

        //Initializes the LogController
        LogController.init();

        //Setting up the IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU tempIMU = hardwareMap.get(IMU.class, "imu 1");
        tempIMU.initialize(new IMU.Parameters(orientationOnRobot));
        tempIMU.resetYaw();
        imu = new MultiIMU.Builder(tempIMU)
                .loggingTarget(MultiIMU.Axis.YAW)
                .build();
        LogController.addLogger(imu);

        //Setting up the MotorControllers for the DriveTrain
        frontRight = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northeastMotor")))
                .driveOrientation(DriveTrain.Orientation.RIGHT_FRONT)
                .id("FrontRight")
                .direction(Motor.Direction.REVERSE)
                .build();
        backRight = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southeastMotor")))
                .driveOrientation(DriveTrain.Orientation.RIGHT_BACK)
                .id("BackRight")
                .direction(Motor.Direction.REVERSE)
                .build();
        frontLeft = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northwestMotor")))
                .driveOrientation(DriveTrain.Orientation.LEFT_FRONT)
                .id("FrontLeft")
                .build();
        backLeft = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southwestMotor")))
                .driveOrientation(DriveTrain.Orientation.LEFT_BACK)
                .id("BackLeft")
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
        LogController.addLogger(odometry);

        //Initializing the DriveTrain
        drive = new DriveTrain.Builder()
                .motor(frontRight, DriveTrain.Orientation.RIGHT_FRONT)
                .motor(frontLeft, DriveTrain.Orientation.LEFT_FRONT)
                .motor(backRight, DriveTrain.Orientation.RIGHT_BACK)
                .motor(backLeft, DriveTrain.Orientation.LEFT_BACK)
                .driveType(DriveTrain.DriveType.MECANUM)
                .build();

        //Setting up the MotorControllers that are not part of the DriveTrain
        nonDriveMotors = new HashMap<>();
        //Add MotorControllers like so:
       /* nonDriveMotors.put("ID",
       new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "motorController1")))
               .id("ID")
               .addLogTarget(MotorController.LogTarget.POSITION)
               .build());*/
        nonDriveMotors.put("Shooter",
                new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "Shooter")))
                        .id("Shooter")
                        .addLogTarget(MotorController.LogTarget.VELOCITY)
                        .ticksPerRevolution(28)
                        .build());
        LogController.addLogger(nonDriveMotors.get("Shooter"));
        nonDriveMotors.put("Intake",
                new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "Intake")))
                        .id("Intake")
                        .addLogTarget(MotorController.LogTarget.VELOCITY)
                        .ticksPerRevolution(288)
                        .build());
        LogController.addLogger(nonDriveMotors.get("Intake"));

        //Currently unused Slider code
        /*nonDriveMotors.put("Slider",
                new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "Slider")))
                        .id("Slider")
                        .addLogTarget(MotorController.LogTarget.VELOCITY)
                        .ticksPerRevolution(288)
                        .build()); */

        nonDriveMotors.put("Advancer",
                new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "Advancer")))
                        .id("Advancer")
                        .addLogTarget(MotorController.LogTarget.VELOCITY)
                        .ticksPerRevolution(288)
                        .build());

        //Currently unused gate code
        /*LogController.addLogger(nonDriveMotors.get("Slider"));
        nonDriveMotors.put("Gate",
                new MotorController.Builder(new CRServoFrame(hardwareMap.get(CRServo.class, "Gate")))
                        .id("Gate")
                        .build()); */

        controller1 = new Controller(gamepad1, 0.0f, "1",
                new Controller.Key[] {
                        Controller.Key.LEFT_STICK_X,
                        Controller.Key.LEFT_STICK_Y,
                        Controller.Key.RIGHT_STICK_X,
                        Controller.Key.RIGHT_STICK_Y
                });

        controller2 = new Controller(gamepad2, 0.0f, "2",
                new Controller.Key[] {
                        Controller.Key.LEFT_STICK_X,
                        Controller.Key.LEFT_STICK_Y,
                        Controller.Key.RIGHT_STICK_X,
                        Controller.Key.RIGHT_STICK_Y
                });
        controller2.createChord("ShooterLock", Controller.Key.LEFT_TRIGGER, Controller.Key.RIGHT_TRIGGER);

        LogController.logInfo("Waiting for start...");

        waitForStart();
        LogController.logInfo("Starting TeleOp.");
        while (opModeIsActive()) {
            //Logs data from all MotorControllers, the imu, and odometry
            LogController.logData();
            PIDController.update();

            //Uses the joysticks to drive the robot with fieldOrientedMecanumDrive
            /*float powLX = controller1.analogDeadband(Controller.Key.LEFT_STICK_X);
            float powRX = controller1.analogDeadband(Controller.Key.RIGHT_STICK_X);
            float powLY = controller1.analogDeadband(Controller.Key.LEFT_STICK_Y);
            frontRight.setPower(powLY - powLX + powRX);
            frontLeft.setPower(powLY + powLX - powRX);
            backRight.setPower(powLY + powLX + powRX);
            backLeft.setPower(powLY - powLX - powRX);*/
            drive.fieldOrientedMecanumDrive(controller1.analogDeadband(Controller.Key.RIGHT_STICK_X), new Vector(controller1.analogDeadband(Controller.Key.LEFT_STICK_X), -1 * controller1.analogDeadband(Controller.Key.LEFT_STICK_Y)), imu.getYaw());

            telemetry.addData("Yaw", imu.getYaw().degree());
            telemetry.addData("NE", frontRight.getPower());
            telemetry.addData("NW", frontLeft.getPower());
            telemetry.addData("SE", backRight.getPower());
            telemetry.addData("SW", backLeft.getPower());

            if (controller2.buttonSingle("ShooterLock")) {
                controller2.flipToggle("ShooterLock");
                shooterLockPower = Math.min(0, controller2.analogDeadband(Controller.Key.RIGHT_STICK_Y));
            }
            if (controller2.getToggle("ShooterLock")) {
                nonDriveMotors.get("Shooter").setPower(shooterLockPower);
            } else {
                nonDriveMotors.get("Shooter").setPower(Math.min(0, controller2.analogDeadband(Controller.Key.RIGHT_STICK_Y)));
            }

            nonDriveMotors.get("Intake").setPower(-1 * controller2.getAnalog(Controller.Key.LEFT_STICK_Y));

            //nonDriveMotors.get("Slider").setPower(controller2.getButton(Controller.Key.X) ? 0.5 : controller2.getButton(Controller.Key.B) ? -0.5 : 0.0);

            nonDriveMotors.get("Advancer").setPower(controller2.getButton(Controller.Key.A) ? 0.5 : controller2.getButton(Controller.Key.Y) ? -0.5 : 0.0);

            /*if (controller2.getButton(Controller.Key.UP)) {
                nonDriveMotors.get("Gate").setPower(1);
            } else if (controller2.getButton(Controller.Key.DOWN)) {
                nonDriveMotors.get("Gate").setPower(-1);
            } else {
                nonDriveMotors.get("Gate").setPower(0);
            } */

            //telemetry.addData("Shooter", controller2.getButton(Controller.Key.BUMPER_LEFT) && controller2.getButton(Controller.Key.BUMPER_RIGHT)) ? "Locked" : "Unlocked");
            telemetry.addData("Shooter Power", nonDriveMotors.get("Shooter").getPower());
            telemetry.addData("Shooter RPM", nonDriveMotors.get("Shooter").getRPM());
            telemetry.addData("Shooter Lock", shooterLockPower);

            telemetry.update();
        }
        //Closes all logs
        LogController.logInfo("TeleOp complete.");
        LogController.closeLogs();
    }
}