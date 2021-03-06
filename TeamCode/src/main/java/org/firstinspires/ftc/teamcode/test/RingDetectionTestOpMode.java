package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpMode;
import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;

@Autonomous(name = "Mecannum: RingDetectionTestOpMode", group = "Mecannum")
@Disabled
public class RingDetectionTestOpMode extends AbstractAutonomousOpModeRR {
    public RingDetectionTestOpMode() {
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
