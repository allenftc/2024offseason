package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricMecanumTeleop extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        initialize();

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.


        // Retrieve the IMU from the hardware map


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            chub.clearBulkCache();
            double y = -gamepad1.left_stick_y*0.5; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x*0.5;
            double rx = gamepad1.right_stick_x*0.5;
            drive(x,y,rx);
            odometry.update();

            telemetry.addData("fl", fl.getCurrentPosition());
            telemetry.addData("bl", bl.getCurrentPosition());
            telemetry.addData("fr", fr.getCurrentPosition());
            telemetry.addData("br", br.getCurrentPosition());
            telemetry.addData("pose", odometry.getPose());
            if (gamepad1.options) {
                resetIMU();
            }
            telemetry.update();
        }
    }
}