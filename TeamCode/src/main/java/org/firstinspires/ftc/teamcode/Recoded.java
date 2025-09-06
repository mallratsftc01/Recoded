package org.firstinspires.ftc.teamcode;

import com.epra.epralib.ftclib.control.Controller;
import com.epra.epralib.ftclib.location.IMUExpanded;
import com.epra.epralib.ftclib.movement.DcMotorExFrame;
import com.epra.epralib.ftclib.movement.DriveTrain;
import com.epra.epralib.ftclib.movement.MotorController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import java.io.IOException;
import java.util.HashMap;

@TeleOp
public class Recoded extends LinearOpMode {

    private MotorController frontLeft;
    private MotorController frontRight;
    private MotorController backLeft;
    private MotorController backRight;
    private DriveTrain drive;

    private HashMap<String, MotorController> nonDriveMotors;

    private IMUExpanded imu;

    private Controller controller1;

    @Override
    public void runOpMode() throws InterruptedException {

        try {
            //Setting up the MotorControllers for the DriveTrain
            frontRight = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northeastMotor")), "front_right");
            frontLeft = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northwestMotor")), "front_left");
            backRight = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southeastMotor")), "back_right");
            backLeft = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southwestMotor")), "back_left");
            //Initializing the DriveTrain
            drive = new DriveTrain(new MotorController[] {frontLeft, frontRight, backLeft, backRight},
                    new DriveTrain.Orientation[] {DriveTrain.Orientation.LEFT_FRONT, DriveTrain.Orientation.RIGHT_FRONT, DriveTrain.Orientation.LEFT_BACK, DriveTrain.Orientation.RIGHT_BACK},
                    DriveTrain.DriveType.MECANUM);

            //Setting up the MotorControllers that are not part of the DriveTrain
            nonDriveMotors = new HashMap<>();
            //Add MotorControllers like so:
            //nonDriveMotors.put(new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, MOTOR_NAME)), ID), ID);

            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            //Setting up the IMU
            IMU tempIMU = hardwareMap.get(IMU.class, "imu 1");
            tempIMU.initialize(new IMU.Parameters(orientationOnRobot));
            imu = new IMUExpanded(tempIMU);

            //Setting up the controller
            controller1 = new Controller(gamepad1, 0.05f, "1");

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
            } catch (IOException e) {
                throw new RuntimeException(e);
            }

            //Uses the joysticks to drive the robot with fieldOrientedMecanumDrive
            drive.fieldOrientedMecanumDrive(controller1.analogDeadband(Controller.Key.RIGHT_STICK_X), controller1.analogDeadband(Controller.Stick.LEFT_STICK), imu.getYaw());

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
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
