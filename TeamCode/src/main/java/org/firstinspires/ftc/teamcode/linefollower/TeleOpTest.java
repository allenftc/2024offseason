package org.firstinspires.ftc.teamcode.linefollower;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.DriveCommand;
import org.firstinspires.ftc.teamcode.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.OTOSSubsystem;

@TeleOp

public class TeleOpTest extends CommandOpMode {
    DcMotor fr, fl, br, bl;
    @Override
    public void initialize() {
        OTOSSubsystem otos = new OTOSSubsystem(hardwareMap,"otos",telemetry);
        otos.reset();
        GamepadEx driver = new GamepadEx(gamepad1);
        fl = hardwareMap.dcMotor.get("frontLeft");
        fr = hardwareMap.dcMotor.get("frontRight");
        bl = hardwareMap.dcMotor.get("backLeft");
        br = hardwareMap.dcMotor.get("backRight");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        MecanumDriveSubsystem mecanum = new MecanumDriveSubsystem(fr, fl, br, bl,telemetry);
        register(otos, mecanum);
        mecanum.setDefaultCommand(new DriveCommand(mecanum, otos, driver::getLeftX,driver::getLeftY,driver::getRightX));
    }
}
