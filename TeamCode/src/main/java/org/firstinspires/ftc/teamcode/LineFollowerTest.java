package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class LineFollowerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        LineFollower lineFollower = hardwareMap.get(LineFollower.class,"linefollower");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("position",lineFollower.getPositionInches());
            telemetry.update();
        }
    }
}
