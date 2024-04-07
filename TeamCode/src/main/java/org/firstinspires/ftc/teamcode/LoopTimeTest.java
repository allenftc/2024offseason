package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class LoopTimeTest extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        initialize();
        ElapsedTime timer = new ElapsedTime();
        startIMUThread(this);
        while (!isStopRequested()) {
            chub.clearBulkCache();
            fl.setPower(Math.random());
            fr.setPower(Math.random());
            bl.setPower(Math.random());
            br.setPower(Math.random());

            telemetry.addData("loop time", timer.milliseconds());
            telemetry.addData("imu angle", imuAngle);
            telemetry.addData("goofy loop time", goofyLoopTime);
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }

            telemetry.update();
            timer.reset();

        }
    }
}
