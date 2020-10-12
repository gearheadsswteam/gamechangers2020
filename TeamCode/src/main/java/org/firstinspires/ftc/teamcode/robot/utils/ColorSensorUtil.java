package org.firstinspires.ftc.teamcode.robot.utils;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.robot.actionparts.SkystoneDetector;

public class ColorSensorUtil implements SkystoneDetector {
    private final static double  SKYSTONE_COLOR_INDEX = 2;
    private ColorSensor colorSensor;
    private DistanceSensor sensorDistance;

    public ColorSensorUtil(ColorSensor sensorColor, DistanceSensor distanceSensor) {
        this.colorSensor = sensorColor;
        this.sensorDistance = distanceSensor;
    }

    public boolean targetsAreVisible() {
        double cc = (colorSensor.red() * colorSensor.green()) / (colorSensor.blue()*colorSensor.blue());
        return cc <= SKYSTONE_COLOR_INDEX;
    }

    private double getCCValue(ColorSensor colorSensor) {
        return (colorSensor.red() * colorSensor.green()) / (colorSensor.blue()*colorSensor.blue());
    }

    private double getCCOpenOrSS(ColorSensor colorSensor) {
        return Math.round((colorSensor.red() * colorSensor.green() * colorSensor.blue())/10000);
    }

}
