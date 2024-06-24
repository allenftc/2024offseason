package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class BabyRobotTest extends LinearOpMode {
    DcMotor left, right;
    @Override
    public void runOpMode() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        waitForStart();
        while(opModeIsActive()) {
            double forward = -gamepad1.right_stick_x;
            double turn = -gamepad1.left_stick_y;
            double l = forward+turn;
            double r = forward-turn;
            double[] powers = {l, r};
            double[] values = Utils.normalize(powers,1);
            left.setPower(values[0]);
            right.setPower(values[1]);
        }

    }

}
