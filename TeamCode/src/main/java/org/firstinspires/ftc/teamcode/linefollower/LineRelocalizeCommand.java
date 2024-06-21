package org.firstinspires.ftc.teamcode.linefollower;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.OTOSSubsystem;
import org.firstinspires.ftc.teamcode.Utils;

public class LineRelocalizeCommand extends CommandBase {
    LineFollowerSubsystem lf;
    OTOSSubsystem otos;
    public LineRelocalizeCommand(LineFollowerSubsystem lineFollowerSubsystem, OTOSSubsystem otosSubsystem) {
        lf = lineFollowerSubsystem;
        otos = otosSubsystem;
        //addRequirements(lf, otos);
    }
    @Override
    public void execute() {
        if (lf.isOnline())
            otos.setPosition(50.0-lf.getPositionInches(),otos.getPose().y);
    }
    @Override
    public boolean isFinished() {
        return Utils.compare(lf.getPositionInches(),0,0.5);
        //return false;
    }
}
