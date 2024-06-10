package org.firstinspires.ftc.teamcode.linefollower;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LineFollowerSubsystem extends SubsystemBase {
    LineFollower lineFollower;
    @Override
    public void periodic() {
        lineFollower.scan();
    }
    public LineFollowerSubsystem(HardwareMap hMap, String key) {
        lineFollower = hMap.get(LineFollower.class,key);
    }
    public double getPosition() {
        return lineFollower.getPosition();
    }
    public double getPositionInches() {
        return lineFollower.getPositionInches();
    }

}
