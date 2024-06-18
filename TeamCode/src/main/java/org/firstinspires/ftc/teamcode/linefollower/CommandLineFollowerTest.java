package org.firstinspires.ftc.teamcode.linefollower;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class CommandLineFollowerTest extends CommandOpMode {
    @Override
    public void initialize() {
        LineFollowerSubsystem subsystem = new LineFollowerSubsystem(hardwareMap, "linefollower");
        register(subsystem);
        subsystem.setDefaultCommand(new ReadLineFollowerCommand(subsystem));
    }
}
