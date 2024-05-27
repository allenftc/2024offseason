package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Odometry {
    private DcMotor fl,  fr,  bl,  br;
    int lastfl = 0;
    int lastfr = 0;
    int lastbl = 0;
    int lastbr = 0;
    double cpr = 537.7*20/30;
    double diameter = 3.78;
    double circumfrence = diameter * Math.PI;
    double ticks_per_inch = (700.0/24.0)*4.0;
    double trackwidth = 22;
    double fx = 0;
    double fy = 0;
    double fRot = 0;
    double dx = 0;
    double dy = 0;
    IMU imu;
    Telemetry telemetry;

    public Odometry(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, IMU imu) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.imu=imu;
    }
    public void update() {
        //encoder deltas
        double dFL = fl.getCurrentPosition()-lastfl;
        dFL /= ticks_per_inch;
        double dFR = fr.getCurrentPosition()-lastfr;
        dFR /= ticks_per_inch;
        double dBL = bl.getCurrentPosition()-lastbl;
        dBL /= ticks_per_inch;
        double dBR = br.getCurrentPosition()-lastbr;
        dBR /= ticks_per_inch;
        //fRot+=(dBR+dFR-dBL+dFL)/(4*2*trackwidth);
        fRot = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        dx = (dFL+dFR+dBL+dBR);
        dy = (dBL+dFR-dFL-dBL);
        fx += Math.cos(fRot)*dx - Math.sin(fRot)*dy;
        fy += Math.sin(fRot)*dx + Math.cos(fRot)*dy;
        lastfl=fl.getCurrentPosition();
        lastfr=fr.getCurrentPosition();
        lastbl=bl.getCurrentPosition();
        lastbr=br.getCurrentPosition();



    }
    public Pose2d getPose() {
        return new Pose2d(fx,fy,new Rotation2d(fRot));
    }
    public void setTelemetry(Telemetry t) {
        telemetry = t;
    }

}
