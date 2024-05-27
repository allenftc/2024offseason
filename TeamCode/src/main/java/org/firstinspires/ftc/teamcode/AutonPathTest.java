package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutonPathTest extends Robot {
    int state = 0;
    Pose2d startPoint = new Pose2d(0,0, new Rotation2d(0));
    Pose2d endPoint = new Pose2d(48,0,new Rotation2d(Math.PI));

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while(opModeIsActive()) {
            chub.clearBulkCache();
            odometry.update();
            if (state == 0) {
                telemetry.addData("target", endPoint);
                driveToPoint(endPoint);
                if (endPoint.getTranslation().getDistance(odometry.getPose().getTranslation())<1) {
                    state = 1;
                }
            }
            else if (state == 1) {
                telemetry.addData("target",startPoint);
                driveToPoint(startPoint);
                if (startPoint.getTranslation().getDistance(odometry.getPose().getTranslation())<1) {
                    state = 0;
                }
            }
            telemetry.addData("fl", fl.getCurrentPosition());
            telemetry.addData("bl", bl.getCurrentPosition());
            telemetry.addData("fr", fr.getCurrentPosition());
            telemetry.addData("br", br.getCurrentPosition());
            telemetry.addData("pose", odometry.getPose());
            telemetry.update();

        }
    }
}
