package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.rotation_kP;
import static org.firstinspires.ftc.teamcode.Robot.translation_kP;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveToPoint extends CommandBase {
    DriveSubsystem drive;
    Pose2d targetPose;
    double tolerance;
    PIDController translationPID = new PIDController(0,0,0);
    PIDController rotationPID = new PIDController(0,0,0);


    public DriveToPoint(DriveSubsystem driveSubsystem) {
        drive = driveSubsystem;
        addRequirements(driveSubsystem);
    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        translationPID.setPID(translation_kP, 0, 0);
        rotationPID.setPID(rotation_kP, 0, 0);
        odometry.update();
        Translation2d move = new Translation2d(fx,fy);
        move.rotateBy(odometry.getPose().getRotation().times(-1));
        drive(-translationPID.calculate(odometry.getPose().getY(),move.getY()),translationPID.calculate(odometry.getPose().getX(),move.getX()),rotationPID.calculate(0, AngleUnit.normalizeDegrees(rot-odometry.getPose().getRotation().getDegrees())));
    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished() {

    }
    public void setTarget(Pose2d pose, double tolerance) {
        this.targetPose = pose;
        this.tolerance = tolerance;
        this.schedule();
    }

}
