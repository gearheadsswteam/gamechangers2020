package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpMode;
import org.firstinspires.ftc.teamcode.robot.utils.Position;

@Autonomous(name = "Mecannum: DriveTestAutonomousCodeTest", group = "Mecannum")
@Disabled
public class DriveTestAutonomousCode  extends AbstractAutonomousOpMode {
    Position initialPosition = new Position(0,0,0);
    Position wobbleGoal_1_Position = new Position(-400,-400,0);

    @Override
    protected void initOpModeAfterStart() {
        //Nothing
    }

    @Override
    protected void executeOpMode() {
//        autonomousRobotMover.moveRobotForwardDistance(0.3, 20);
//
//        sleep(2000);
//
//        autonomousRobotMover.moveRobotBackwardDistance(0.3, 20);

        autonomousRobotMover.moveRobotToPosition(0.3, initialPosition,wobbleGoal_1_Position);
    }
}
