package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.robot.utils.Position;

public class RingCase0AutonomousOpMode extends AbstractAutonomousOpMode{
    Position initialPosition = new Position(4,0,0);
    Position wobbleGoal_1_Position = new Position(4,0,0);
    Position wobbleDropPosition = new Position(5,4,0);
    Position shootingPosition = new Position(5,4,0);
    Position ringPickupPosition = new Position(5,4,0);
    Position wobbleGoal_2_Position = new Position(4,0,0);
    Position parkPosition = new Position(4,0,0);

    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {
        moveFirstWobbleGoal();
        shootPreloadedRings();
        grabRings();
        shootGrabbedRings();
        moveSecondWobbleGoal();
        park();
    }

    private void moveFirstWobbleGoal(){

        autonomousRobotMover.moveRobotToPosition(1,initialPosition,wobbleDropPosition);
    }

    private void shootPreloadedRings(){
        autonomousRobotMover.moveRobotForwardDistance(0.5, 12);
    }

    private void grabRings(){

    }

    private void shootGrabbedRings(){

    }

    private void moveSecondWobbleGoal(){

    }

    private void park(){

    }
}
