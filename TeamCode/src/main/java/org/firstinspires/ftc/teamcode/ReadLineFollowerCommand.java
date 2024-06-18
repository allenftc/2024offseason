package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

public class ReadLineFollowerCommand extends CommandBase {
    LineFollowerSubsystem lf;
    double position;
    public ReadLineFollowerCommand(LineFollowerSubsystem lineFollowerSubsystem) {
        lf = lineFollowerSubsystem;
        addRequirements(lf);
    }
    @Override
    public void execute() {
        position = lf.getPositionInches();
        Log.println(Log.VERBOSE, "linefollower", "Position: " + position);
    }
    public double getPosition() {
        return position;
    }
}
