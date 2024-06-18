package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OTOSSubsystem extends SubsystemBase {
    SparkFunOTOS otos;
    SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D();
    Telemetry t;
    @Override
    public void periodic() {
        pose = otos.getPosition();
        t.addData("x", pose.x);
        t.addData("y", pose.y);
        t.addData("h",pose.h);
        t.update();
    }
    public OTOSSubsystem(HardwareMap hMap, String name) {
        otos = hMap.get(SparkFunOTOS.class, name);
        otos.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        otos.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(4,1.5,90));
        otos.setAngularScalar(360/364.0);
        otos.calibrateImu();
    }
    public OTOSSubsystem(HardwareMap hMap, String name, Telemetry telemetry) {
        otos = hMap.get(SparkFunOTOS.class, name);
        otos.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        otos.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(4,1.5,90));
        otos.setAngularScalar(360/364.0);
        otos.calibrateImu();
        t = telemetry;
    }
    public SparkFunOTOS.Pose2D getPose() {
        return pose;
    }
    public Pose2d getFTCLibPose() {
        return new Pose2d(getPose().x, getPose().y, new Rotation2d(Math.toRadians(getPose().h)));
    }
    public void reset() {
        otos.resetTracking();
    }

}