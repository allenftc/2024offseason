package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class LineFollowerTest extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        LineFollower lineFollower = hardwareMap.get(LineFollower.class,"linefollower");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("raw",Integer.toBinaryString(lineFollower.scan()));
            telemetry.addData("position",lineFollower.getPosition());
            telemetry.addData("inches",lineFollower.getPositionInches());
            telemetry.addData("time",timer.milliseconds());
            telemetry.update();
            timer.reset();
        }
    }
}
