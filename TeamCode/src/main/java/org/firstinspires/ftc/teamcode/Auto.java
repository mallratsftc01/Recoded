package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.epra.epralib.ftclib.control.Controller;
import com.epra.epralib.ftclib.location.MultiIMU;
import com.epra.epralib.ftclib.location.Odometry;
import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Geometry;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.movement.Motor;
import com.epra.epralib.ftclib.movement.frames.CRServoFrame;
import com.epra.epralib.ftclib.movement.frames.DcMotorExFrame;
import com.epra.epralib.ftclib.movement.DriveTrain;
import com.epra.epralib.ftclib.movement.MotorController;
import com.epra.epralib.ftclib.movement.frames.ServoFrame;
import com.epra.epralib.ftclib.movement.pid.PIDController;
import com.epra.epralib.ftclib.storage.autonomous.AutoProgram;
import com.epra.epralib.ftclib.storage.autonomous.AutoStep;
import com.epra.epralib.ftclib.storage.autonomous.MotorControllerAutoModule;
import com.epra.epralib.ftclib.storage.logdata.LogController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends LinearOpMode {

    //These variables lead to the JSON files that control the vast majority of auto
    private final String AUTO_DIRECTORY = "auto";
    private final String PROGRAM_NAME = "no_odometry_program";
    private final String PID_SETTINGS_FILENAME = "pid.json";
    private final String ENCODER_SETTINGS_FILENAME = "encoder.json";
    //The starting position must also be set
    private final Pose START_POSE = new Pose(new Vector(0, 0), new Angle());

    private MotorController frontRight;
    private MotorController backRight;
    private MotorController backLeft;
    private MotorController frontLeft;
    private DriveTrain drive;

    private HashMap<String, MotorController> nonDriveMotors;

    private MultiIMU imu;

    private Odometry odometry;

    private AutoProgram program;

    @Override
    public void runOpMode() {
        //Initializes the LogController
        LogController.init();

        //Setting up the IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU tempIMU = hardwareMap.get(IMU.class, "imu 1");
        tempIMU.initialize(new IMU.Parameters(orientationOnRobot));
        imu = new MultiIMU.Builder(tempIMU)
                .loggingTarget(MultiIMU.Axis.YAW)
                .build();
        LogController.addLogger(imu);

        //Setting up the MotorControllers for the DriveTrain
        frontRight = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northeastMotor")))
                .driveOrientation(DriveTrain.Orientation.RIGHT_FRONT)
                .direction(Motor.Direction.REVERSE)
                .build();
        backRight = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southeastMotor")))
                .driveOrientation(DriveTrain.Orientation.RIGHT_BACK)
                .direction(Motor.Direction.REVERSE)
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
                .useEncoderSettingsFile(ENCODER_SETTINGS_FILENAME)
                .heading(imu::getYaw)
                .startPose(new Pose(new Vector(0, 0), Angle.degree(0)))
                .loggingTargets(Odometry.LoggingTarget.X, Odometry.LoggingTarget.Y)
                .build();
        LogController.addLogger(odometry);

        //Initializing the DriveTrain
        drive = new DriveTrain.Builder()
                .motor(frontRight)
                .motor(frontLeft)
                .motor(backRight)
                .motor(backLeft)
                .driveType(DriveTrain.DriveType.MECANUM)
                .build();

        //Setting up the MotorControllers that are not part of the DriveTrain
        nonDriveMotors = new HashMap<>();
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
        nonDriveMotors.put("Advancer",
                new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "Advancer")))
                        .id("Advancer")
                        .addLogTarget(MotorController.LogTarget.VELOCITY)
                        .ticksPerRevolution(288)
                        .build());
        PIDController.getPIDsFromFile(PID_SETTINGS_FILENAME);


        // Setting up the AutoProgram
        final var autoData = new Object() {
            long startTime = System.currentTimeMillis();
            long stepStartTime = startTime;
            long movementStartTime = startTime;
        };

        AutoProgram.Builder autoBuilder = new AutoProgram.Builder(AUTO_DIRECTORY)
                .programName(PROGRAM_NAME)
                .dataSupplier("Time.Seconds", () -> (double)(System.currentTimeMillis() - autoData.startTime) / 1000.0)
                .dataSupplier("Time.Step.Seconds", () -> (double)(System.currentTimeMillis() - autoData.stepStartTime) / 1000.0)
                .dataSupplier("Time.Movement.Seconds", () -> (double)(System.currentTimeMillis() - autoData.movementStartTime) / 1000.0)

                .dataSupplier("Position.X", () -> odometry.getPose().pos.x())
                .dataSupplier("Position.Y", () -> odometry.getPose().pos.y())
                .dataSupplier("Position.Theta", () -> imu.getYaw().degree())
                .dataSupplier("Velocity.X", () -> odometry.getVelocity().x())
                .dataSupplier("Velocity.Y", () -> odometry.getVelocity().y())
                .dataSupplier("Acceleration.X", () -> odometry.getAcceleration().x())
                .dataSupplier("Acceleration.Y", () -> odometry.getAcceleration().y())

                .dataSupplier("Shooter.Velocity", () -> Math.abs(nonDriveMotors.get("Shooter").getRPM()));

        program = autoBuilder.build();

        LogController.logInfo("Waiting for start...");
        waitForStart();
        LogController.logInfo("Starting Autonomous.");
        autoData.startTime = System.currentTimeMillis();
        autoData.stepStartTime = autoData.startTime;
        autoData.movementStartTime = autoData.startTime;
        while (opModeIsActive()) {
            //Logs
            LogController.logData();

            //Updates all active PID loops
            PIDController.update();

            //Updates the auto program
            if (!program.autoActive()) { break; }
            AutoStep lastStep = program.getCurrentStep();
            String lastMovement = program.getCurrentMovement();
            program.updateStep();
            AutoStep currentStep = program.getCurrentStep();
            if (!lastStep.equals(currentStep)) {
                autoData.stepStartTime = System.currentTimeMillis();
            }
            if (!lastMovement.equals(program.getCurrentMovement())) {
                autoData.movementStartTime = System.currentTimeMillis();
            }

            //Updates the DriveTrain with new instructions
            //drive.useDriveTrainAutoModule(currentStep.driveTrainModule());
            drive.mecanumDrive(0,
                    Geometry.scale(new Vector(currentStep.driveTrainModule().x(),
                                    currentStep.driveTrainModule().y()).unit(),
                            currentStep.driveTrainModule().maxPower()));
            //drive.mecanumDrive(0, new Vector(currentStep.driveTrainModule().x(), currentStep.driveTrainModule().y()).unit());

            //Updates all the MotorControllers with new instructions
            if (currentStep.motorControllerModules() != null) {
                for (MotorControllerAutoModule m : currentStep.motorControllerModules()) {
                    nonDriveMotors.get(m.id()).useMotorControllerAutoModule(m);
                }
            }

            telemetry.addData("Current Movement:", program.getCurrentMovement());
            telemetry.addData("Step Information:", program.getCurrentStep().comment());
            telemetry.addData("Drivetrain X:", drive.driveTrainVector().x());
            telemetry.addData("Drivetrain Y:", drive.driveTrainVector().y());
            telemetry.update();
        }
    }
}