package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Mecannum: GearheadsMecAutoOpModeTest", group = "Mecannum")
public class ExampleAutonomousMode extends AbstractAutonomousOpMode {
    @Override
    protected void initOpModeAfterStart() {
        //Nothing
    }

    @Override
    protected void executeOpMode() {
        autonomousRobotMover.moveRobotForwardDistance(1, 150);

        sleep(2000);

        autonomousRobotMover.moveRobotBackwardDistance(1, 150);
    }
}
