package org.firstinspires.ftc.teamcode;

import androidx.annotation.GuardedBy;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class Robot extends LinearOpMode {
    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    private IMU imu;
    public IMU publicIMU;

    @GuardedBy("imuLock")
    private HuskyLens huskyLens;

    private Thread imuThread;
    public double imuAngle;
    public HuskyLens.Block[] blocks;
    DcMotor fl, fr, bl, br;
    LynxModule chub;
    public double goofyLoopTime = 0;
    public Odometry odometry;
    public static double translation_kP = 0.01;
    PIDController translationPID = new PIDController(0,0,0);
    public static double rotation_kP = 0.0001;
    PIDController rotationPID = new PIDController(0,0,0);
    public double lastx=0;
    public double lasty=0;
    public double lastrot=0;
    SparkFunOTOS otos;
    SparkFunOTOS.Pose2D pose2D = new SparkFunOTOS.Pose2D(0,0,0);
    public void initialize() {
        chub = hardwareMap.getAll(LynxModule.class).get(0);
        chub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        fl = hardwareMap.dcMotor.get("frontLeft");
        fr = hardwareMap.dcMotor.get("frontRight");
        bl = hardwareMap.dcMotor.get("backLeft");
        br = hardwareMap.dcMotor.get("backRight");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        publicIMU = hardwareMap.get(IMU.class, "imu");
        publicIMU.initialize(new IMU.Parameters(orientationOnRobot));
        huskyLens = hardwareMap.get(HuskyLens.class, "husky");
        huskyLens.initialize();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        odometry = new Odometry(fl,fr,bl,br,imu);
        otos = hardwareMap.get(SparkFunOTOS.class,"otos");
        otos.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        otos.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(-6,-0.5,180));
        otos.calibrateImu();
        resetSystems();

    }
    public void resetSystems() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        otos.resetTracking();
    }
    public void resetIMU() {
        imu.resetYaw();
    }
    public void startIMUThread(LinearOpMode opMode) {
        ElapsedTime timer1 = new ElapsedTime();


            imuThread = new Thread(() -> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                    synchronized (imuLock) {
                        goofyLoopTime = timer1.milliseconds();
                        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                        blocks = huskyLens.blocks();
                        timer1.reset();
                    }
                }
            });
            imuThread.start();
    }
    public void drive(double x, double y, double rx) {
        telemetry.addData("drive x", y);
        telemetry.addData("drive y", -x);
        telemetry.addData("drive rx", rx);
        double botHeading = Math.toRadians(pose2D.h);



        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        /*if (!compare(lastx,rotX,0.2)) {
            if (rotX-lastx>0) {
                lastx=rotX+0.1;
            }
            else {
                lastx=rotX-0.1;
            }
        }
        if (!compare(lasty,rotY,0.2)) {
            if (rotY-lasty>0) {
                lasty=rotY+0.1;
            }
            else {
                lasty=rotY-0.1;
            }
        }*/

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        if (!compare(rotX,lastx,0.1)) {

        }

        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);
        lastx = rotX;
        lasty = rotY;
        lastrot = rx;
    }
    public void driveToPoint(double fx, double fy, double rot) {
        /*translationPID.setPID(translation_kP, 0, 0);
        rotationPID.setPID(rotation_kP, 0, 0);
        odometry.update();
        double xError = fx-odometry.getPose().getX();
        double yError = fy-odometry.getPose().getY();
        double distance = Math.sqrt(xError*xError+yError*yError);
        double angleError = Math.atan2(xError,-yError);
        translationPID.setSetPoint(distance);
        double moveX = Math.cos(angleError)*translationPID.calculate();
        moveX = Range.clip(moveX,-0.5,0.5);
        double moveY = Math.sin(angleError)*translationPID.calculate();
        moveY = Range.clip(moveY,-0.5,0.5);
        rotationPID.setSetPoint(AngleUnit.normalizeDegrees(rot-odometry.getPose().getRotation().getDegrees()));
        double moveTurn = rotationPID.calculate();
        drive(moveX,moveY,moveTurn);*/
        translationPID.setPID(translation_kP, 0, 0);
        rotationPID.setPID(rotation_kP, 0, 0);
        odometry.update();
        Translation2d move = new Translation2d(fx,fy);
        move.rotateBy(odometry.getPose().getRotation().times(-1));
        drive(-translationPID.calculate(odometry.getPose().getY(),move.getY()),translationPID.calculate(odometry.getPose().getX(),move.getX()),rotationPID.calculate(0,AngleUnit.normalizeDegrees(rot-odometry.getPose().getRotation().getDegrees())));

    }
    public void driveToPoint(Pose2d pose) {
        driveToPoint(pose.getX(),pose.getY(),pose.getHeading());
    }
    public static boolean compare(double a, double b, double tolerance) {
        return Math.abs(a-b)<tolerance;
    }

}
