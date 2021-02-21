package org.firstinspires.ftc.teamcode.autonomousRR.blueteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;
import org.firstinspires.ftc.teamcode.autonomousRR.redteam.RedTeamPositions;

@Autonomous
public class TestBluePositions extends AbstractAutonomousOpModeRR {
    Pose2d initPos = BlueTeamPositions.INIT_POS;

    @Override
    protected void initOpModeBeforeStart() {
        super.initOpModeBeforeStart();
        mecanumDriveRR.setPoseEstimate(BlueTeamPositions.INIT_POS);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {
        //testMovementinAllDirection();
        //testSplineMovement();
        testStraightLine();
    }

    private void testStraightLine(){
        mecanumDriveRR.setPoseEstimate(new Pose2d(0,0,0));
        Trajectory trajectory = mecanumDriveRR.trajectoryBuilder(new Pose2d(0,0,0), 0).forward(48).build();
        mecanumDriveRR.followTrajectory(trajectory);
    }

}
