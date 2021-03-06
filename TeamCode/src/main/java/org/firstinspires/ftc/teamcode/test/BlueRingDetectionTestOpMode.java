package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;

@Autonomous(name = "BlueRingDetectionTestOpMode", group = "Blue")
public class BlueRingDetectionTestOpMode extends AbstractAutonomousOpModeRR {
    public BlueRingDetectionTestOpMode() {
        super.TEAM_TYPE = AbstractAutonomousOpModeRR.BLUE_TEAM;
    }

    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {
        while (opModeIsActive()) {
            //robot.ringDetector.printRings();
            int rings = robot.ringDetector.detectRings();
            telemetry.addData("Rings ", rings);
            telemetry.update();
            sleep(1000);
        }
    }
}
