package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.epra.epralib.ftclib.control.Controller;
import com.epra.epralib.ftclib.location.MultiIMU;
import com.epra.epralib.ftclib.location.Odometry;
import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.movement.DriveTrain;
import com.epra.epralib.ftclib.movement.Motor;
import com.epra.epralib.ftclib.movement.MotorController;
import com.epra.epralib.ftclib.movement.frames.DcMotorExFrame;
import com.epra.epralib.ftclib.movement.pid.PIDController;
import com.epra.epralib.ftclib.storage.autonomous.AutoProgram;
import com.epra.epralib.ftclib.storage.autonomous.AutoStep;
import com.epra.epralib.ftclib.storage.logdata.LogController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.util.HashMap;
import java.util.function.Supplier;

@Config
class PIDInput {
    public static PIDCoefficients ANGLE_COEFFICIENTS = new PIDCoefficients(0.0, 0.0, 0.0);
    public static PIDCoefficients POINT_COEFFICIENTS = new PIDCoefficients(0.0, 0.0, 0.0);
    public static PIDCoefficients MOTION_COEFFICIENTS = new PIDCoefficients(0.0, 0.0, 0.0);
}

@TeleOp(name = "DriveTrainPIDCalibrationLocal", group = "Calibration")
public class DriveTrainPIDCalibration extends LinearOpMode {

    //These variables lead to the JSON files that control the vast majority of auto
    private final String DIRECTORY = "DT_PID_Calibration";
    private final String PID_SETTINGS_FILENAME = "pid.json";
    private final String ENCODER_SETTINGS_FILENAME = "encoder.json";
    //The starting position must also be set
    private final Pose START_POSE = new Pose(new Vector(0, 0), new Angle());

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private MotorController frontRight;
    private MotorController backRight;
    private MotorController backLeft;
    private MotorController frontLeft;
    private DriveTrain drive;

    private MultiIMU imu;
    private Odometry odometry;

    private Controller controller1;

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
        tempIMU.resetYaw();
        imu = new MultiIMU.Builder(tempIMU)
                    .loggingTarget(MultiIMU.Axis.YAW)
                    .build();
        imu.recenter();
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
        frontRight.zero();

        //Setting up the Odometry
        odometry = new Odometry.Builder()
                .leftEncoder(frontLeft::getCurrentPosition, 0.01, new Vector(8, 4))
                .rightEncoder(frontRight::getCurrentPosition, 0.01, new Vector(-8, 4))
                .perpendicularEncoder(backLeft::getCurrentPosition, 0.01, new Vector(0, 2))
                .useEncoderSettingsFile(ENCODER_SETTINGS_FILENAME)
                .heading(imu::getYaw)
                .startPose(START_POSE)
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
                .poseSupplier(odometry::getPose)
                .deltaPoseSupplier(odometry::getDeltaPose)
                .build();

        /*HashMap<String, PIDCoefficients> pidMap = PIDController.getPIDsFromFile(PID_SETTINGS_FILENAME);
        PIDController.addPID("DriveTrain_P", pidMap.get("DriveTrain_P"), drive::getPointError);
        PIDController.addPID("DriveTrain_A", pidMap.get("DriveTrain_A"), drive::getAngleError);
        PIDController.addPID("DriveTrain_M", pidMap.get("DriveTrain_M"), drive::getMotionError);*/

        controller1 = new Controller(gamepad1, 0.05f, "Controller1");

        // Setting up the AutoProgram
        var ref = new Object() {
            long startTime = 0;
            long movementStartTime = 0;
            boolean atTarget = false;
        };
        
        HashMap<String, Supplier<Double>> dataSuppliers = new HashMap<>();
        dataSuppliers.put("Time.Seconds", () -> (double)(System.currentTimeMillis() - ref.startTime) / 1000.0);
        dataSuppliers.put("Time.Movement.Seconds", () -> (double)(System.currentTimeMillis() - ref.movementStartTime) / 1000.0);
        dataSuppliers.put("Position.X", () -> odometry.getPose().pos.x());
        dataSuppliers.put("Position.Y", () -> odometry.getPose().pos.y());
        dataSuppliers.put("Position.Theta", () -> imu.getYaw().degree());
        dataSuppliers.put("Position.AtTarget", () -> ref.atTarget ? 1.0 : 0.0);
        dataSuppliers.put("Velocity.X", () -> odometry.getVelocity().x());
        dataSuppliers.put("Velocity.Y", () -> odometry.getVelocity().y());
        dataSuppliers.put("Acceleration.X", () -> odometry.getAcceleration().x());
        dataSuppliers.put("Acceleration.Y", () -> odometry.getAcceleration().y());
        dataSuppliers.put("Controller.A", () -> controller1.getButton(Controller.Key.A) ? 1.0 : 0.0);
        dataSuppliers.put("Controller.B", () -> controller1.getButton(Controller.Key.B) ? 1.0 : 0.0);
        dataSuppliers.put("Controller.X", () -> controller1.getButton(Controller.Key.X) ? 1.0 : 0.0);
        dataSuppliers.put("Controller.Y", () -> controller1.getButton(Controller.Key.Y) ? 1.0 : 0.0);

        program = new AutoProgram(DIRECTORY, new HashMap<>(dataSuppliers));

        LogController.logInfo("Waiting for start...");
        waitForStart();
        LogController.logInfo("Starting Calibration");
        ref.startTime = System.currentTimeMillis();
        ref.movementStartTime = System.currentTimeMillis();
        while (opModeIsActive()) {
            //Logs
            LogController.logData();

            //Updates all active PID loops
            drive.tuneAnglePID(PIDInput.ANGLE_COEFFICIENTS);
            drive.tunePointPID(PIDInput.POINT_COEFFICIENTS);
            drive.tuneMotionPID(PIDInput.MOTION_COEFFICIENTS);
            PIDController.update();

            //Updates the auto program
            String last = program.getCurrentMovement();
            program.updateStep();
            AutoStep currentStep = program.getCurrentStep();
            if (last != null && !last.equals(program.getCurrentMovement())) {
                ref.movementStartTime = System.currentTimeMillis();
            }

            //Updates the DriveTrain with new instructions
            ref.atTarget = drive.useDriveTrainAutoModule(currentStep.driveTrainModule());

            TelemetryPacket telemetryPacket = new TelemetryPacket();
            telemetryPacket.put("Step", currentStep.comment());
            telemetryPacket.put("End Condition Met", program.currentStepEndCondition());
            telemetryPacket.put("Time", (double)(System.currentTimeMillis() - ref.startTime) / 1000.0);
            telemetryPacket.put("Time.Movement", (double)(System.currentTimeMillis() - ref.movementStartTime) / 1000.0);
            telemetryPacket.addLine(" ");
            telemetryPacket.put("Position", odometry.getPose());
            telemetryPacket.put("Position Error", drive.getPointError());
            telemetryPacket.put("Position Error Magnitude", drive.getPointError().length());
            telemetryPacket.put("Target", currentStep.driveTrainModule().targetPose());
            telemetryPacket.put("Left Encoder", frontLeft.getCurrentPosition());
            telemetryPacket.put("Right Encoder", frontRight.getCurrentPosition());
            telemetryPacket.put("Perpendicular Encoder", backLeft.getCurrentPosition());
            dashboard.sendTelemetryPacket(telemetryPacket);
        }
    }
}