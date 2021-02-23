package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpMode;
import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;

@Autonomous(name = "Mecannum: RingDetectionTestOpMode", group = "Mecannum")
public class RingDetectionTestOpMode extends AbstractAutonomousOpModeRR {

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
