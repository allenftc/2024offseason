package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.linefollower.LineFollowerSubsystem;
import org.firstinspires.ftc.teamcode.linefollower.LineRelocalizeCommand;
@TeleOp

public class RelocalizeTest extends CommandOpMode {
    @Override
    public void initialize() {
        OTOSSubsystem otos = new OTOSSubsystem(hardwareMap,"otos",telemetry);
        LineFollowerSubsystem lineFollowerSubsystem = new LineFollowerSubsystem(hardwareMap,"linefollower");
        register(otos, lineFollowerSubsystem);
        lineFollowerSubsystem.setDefaultCommand(new LineRelocalizeCommand(lineFollowerSubsystem,otos));
    }
}
