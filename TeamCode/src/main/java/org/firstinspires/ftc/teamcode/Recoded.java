package org.firstinspires.ftc.teamcode;

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

        try {
            //Setting up the IMU
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            IMU tempIMU = hardwareMap.get(IMU.class, "imu 1");
            tempIMU.initialize(new IMU.Parameters(orientationOnRobot));
            imu = new MultiIMU(tempIMU);

            //Setting up the MotorControllers for the DriveTrain
            frontRight = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northeastMotor")), "front_right");
            frontRight.setDirection(Motor.Direction.REVERSE);
            frontLeft = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northwestMotor")), "front_left");
            backRight = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southeastMotor")), "back_right");
            backRight.setDirection(Motor.Direction.REVERSE);
            backLeft = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southwestMotor")), "back_left");

            //Setting up the Odometry
            odometry = new Odometry(frontLeft::getCurrentPosition, frontRight::getCurrentPosition, backLeft::getCurrentPosition,
                    new Vector(7.92784216, 3.75),
                    new Vector(-8, 3.75),
                    new Vector(0, 2.0),
                    imu::getYaw,
                    START_POSE
            );

            //Initializing the DriveTrain
            drive = new DriveTrain(new MotorController[] {frontLeft, frontRight, backLeft, backRight},
                    new DriveTrain.Orientation[] {DriveTrain.Orientation.LEFT_FRONT, DriveTrain.Orientation.RIGHT_FRONT, DriveTrain.Orientation.LEFT_BACK, DriveTrain.Orientation.RIGHT_BACK},
                    odometry::getPose,
                    odometry::getDeltaPose,
                    DriveTrain.DriveType.MECANUM);

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

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        waitForStart();
        while (opModeIsActive()) {
            //Logs data from all MotorControllers, the imu, and odometry
            try {
                frontRight.log();
                frontLeft.log();
                backRight.log();
                backLeft.log();
                for (MotorController m : nonDriveMotors.values()) {
                    m.log();
                }
                imu.log();
                controller1.log();
                controller2.log();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
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
                    shooterLockPower = Math.max(0, controller2.analogDeadband(Controller.Key.RIGHT_STICK_Y));
                }
                nonDriveMotors.get("Shooter").setPower(shooterLockPower);
            }
            else {
                nonDriveMotors.get("Shooter").setPower(Math.max(0, controller2.analogDeadband(Controller.Key.RIGHT_STICK_Y)));
                shooterLockPower = 0;
            }
            nonDriveMotors.get("Intake").setPower(controller2.analogDeadband(Controller.Key.LEFT_STICK_Y));
            if (controller2.getButton(Controller.Key.UP) && (controller2.getButton(Controller.Key.RIGHT_STICK_Y) || controller2.buttonToggleSingle("shooterLock"))) {
                nonDriveMotors.get("Advancer").setPower(-1);
            } else{
                nonDriveMotors.get("Advancer").setPower(0);
            }

            telemetry.addData("Shooter", (controller2.buttonToggleSingle("shooterLock")) ? "Locked" : "Unlocked");
            try {
                telemetry.addData("Shooter Velocity", nonDriveMotors.get("Shooter").getVelocity());
            } catch (Exception e) {
                telemetry.addData("Error", e.toString());
            }
            telemetry.update();

        }
        //Closes all logs
        try {
            frontRight.closeLog();
            frontLeft.closeLog();
            backRight.closeLog();
            backLeft.closeLog();
            for (MotorController m : nonDriveMotors.values()) {
                m.closeLog();
            }
            imu.closeLog();
            controller1.closeLog();
            controller2.closeLog();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}