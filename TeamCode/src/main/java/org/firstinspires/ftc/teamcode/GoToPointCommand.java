package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config

public class GoToPointCommand extends CommandBase {
    public static PIDController translationalPID = new PIDController(0.03,0,0);
    public static PIDController headingPID = new PIDController(0.001,0,0);
    public static double translationkP = 0.02;
    public static double translationkI = 0.01;
    public static double translationkD = 0.00000;
    public static double headingkP = 0.002;
    public static double headingkI = 0.0003;
    public static double headingkD = 0;
    MecanumDriveSubsystem drive;
    OTOSSubsystem otos;
    Pose2d target;
    Telemetry t = FtcDashboard.getInstance().getTelemetry();
    double tol = 1;
    double hTol = 0.1;


    public GoToPointCommand(MecanumDriveSubsystem driveSubsystem, OTOSSubsystem otosSubsystem, Pose2d targetPose) {
        drive = driveSubsystem;
        otos = otosSubsystem;
        target = targetPose;
        addRequirements(drive, otos);
    }
    public GoToPointCommand(MecanumDriveSubsystem driveSubsystem, OTOSSubsystem otosSubsystem, Pose2d targetPose, double tolerance) {
        drive = driveSubsystem;
        otos = otosSubsystem;
        target = targetPose;
        addRequirements(drive, otos);
        tol=tolerance;
    }
    public GoToPointCommand(MecanumDriveSubsystem driveSubsystem, OTOSSubsystem otosSubsystem, Pose2d targetPose, double tolerance, double hTolerance) {
        drive = driveSubsystem;
        otos = otosSubsystem;
        target = targetPose;
        addRequirements(drive, otos);
        tol=tolerance;
        hTol = hTolerance;
    }
    public GoToPointCommand(MecanumDriveSubsystem driveSubsystem, OTOSSubsystem otosSubsystem, Pose2d targetPose, Telemetry telemetry) {
        drive = driveSubsystem;
        otos = otosSubsystem;
        target = targetPose;
        t = telemetry;
        addRequirements(drive, otos);
    }
    @Override
    public void execute() {
        translationalPID.setPID(translationkP,translationkI,translationkD);
        headingPID.setPID(headingkP,headingkI,headingkD);
        Pose2d currentPose = otos.getFTCLibPose();
        double xDist = target.getX() - currentPose.getX();
        double yDist = target.getY() - currentPose.getY();
        double angle = Math.atan2(yDist,xDist);
        double magnitude = Math.sqrt(Math.hypot(xDist,yDist));
        magnitude = translationalPID.calculate(0,magnitude);
        double xMove = Math.cos(angle)*magnitude;
        double yMove = Math.sin(angle)*magnitude;

        drive.drive(xMove,yMove, headingPID.calculate(0,-AngleUnit.normalizeDegrees(target.getRotation().getDegrees()-currentPose.getRotation().getDegrees())),currentPose.getRotation().getDegrees());
    }
    @Override
    public void end(boolean wasInterrupted) {
        drive.drive(0,0,0,0);
    }
    @Override
    public boolean isFinished() {
        return target.getTranslation().getDistance(otos.getFTCLibPose().getTranslation())<tol &&
                Utils.compare(target.getHeading(),otos.getFTCLibPose().getHeading(),hTol);
    }
}
