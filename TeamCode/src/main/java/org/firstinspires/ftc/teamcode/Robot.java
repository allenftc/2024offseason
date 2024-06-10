package org.firstinspires.ftc.teamcode;

import androidx.annotation.GuardedBy;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
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
    public void initialize() {
        chub = hardwareMap.getAll(LynxModule.class).get(0);
        chub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        fl = hardwareMap.dcMotor.get("frontLeft");
        fr = hardwareMap.dcMotor.get("frontRight");
        bl = hardwareMap.dcMotor.get("backLeft");
        br = hardwareMap.dcMotor.get("backRight");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        publicIMU = hardwareMap.get(IMU.class, "imu");
        publicIMU.initialize(new IMU.Parameters(orientationOnRobot));
        huskyLens = hardwareMap.get(HuskyLens.class, "husky");
        huskyLens.initialize();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


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


}
