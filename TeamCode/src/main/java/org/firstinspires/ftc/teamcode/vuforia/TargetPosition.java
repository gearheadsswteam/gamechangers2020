package org.firstinspires.ftc.teamcode.vuforia;

public class TargetPosition {

    private String targetName;     // Name of the currently tracked target
    private double robotX;         // X displacement from target center
    private double robotBearing;  // Heading of the target , relative to the robot's unrotated center


    public TargetPosition(String targetName, double robotX, double robotBearing) {
        this.targetName = targetName;
        this.robotX = robotX;
        this.robotBearing = robotBearing;
    }

    public double getRobotX() {
        return -robotX;
    }

    public String getTargetName() {
        return targetName;
    }

    public double getRobotBearing() {
        return robotBearing;
    }
}
