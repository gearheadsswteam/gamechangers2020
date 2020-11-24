package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpMode;
import org.firstinspires.ftc.teamcode.odometry.GEarheadsOdometryPositionFinder;
import org.firstinspires.ftc.teamcode.odometry.RobotPositionFinderFactory;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobot;

@Autonomous(name = "Mecannum: OdometryTestOpMode", group = "Mecannum")
public class OdometryTestOpMode extends AbstractAutonomousOpMode {
    private GEarheadsOdometryPositionFinder globalPositionUpdate;

    @Override
    protected void initOpModeAfterStart() {
        globalPositionUpdate = robot.globalPositionUpdate;
    }

    @Override
    protected void executeOpMode() {

        //Display Global (x, y, theta) coordinates
        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinateInInches());
        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinateInInches());
        telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
        telemetry.update();
    }
}
