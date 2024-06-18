package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    MecanumDriveSubsystem drive;
    OTOSSubsystem otos;
    DoubleSupplier x, y, rx, heading;
    public DriveCommand(MecanumDriveSubsystem driveSubsystem, OTOSSubsystem otos, DoubleSupplier inputX, DoubleSupplier inputY, DoubleSupplier inputRx) {
        this.drive = driveSubsystem;
        this.otos = otos;
        this.x = inputX;
        this.y = inputY;
        this.rx = inputRx;
        addRequirements(drive, otos);
    }
    @Override
    public void execute() {
        drive.drive(-y.getAsDouble(),x.getAsDouble(),rx.getAsDouble(),otos.getPose().h);
    }
}
