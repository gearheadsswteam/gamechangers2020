package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;

@Autonomous(name = "RedRingDetectionTestOpMode", group = "Red")
public class RedRingDetectionTestOpMode extends AbstractAutonomousOpModeRR {
    public RedRingDetectionTestOpMode() {
        super.TEAM_TYPE = AbstractAutonomousOpModeRR.RED_TEAM;
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
