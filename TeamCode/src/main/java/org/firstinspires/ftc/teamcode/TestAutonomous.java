package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpModeRR;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import static java.lang.Math.*;
@Autonomous
public class TestAutonomous extends AbstractAutonomousOpModeRR {
    double ringRedThreshold = 1.1;
    int ringNum;
    Point ringRectBL = new Point(50, 10);
    Point ringRectUR = new Point(90, 90);
    Point ringRectML = new Point(80, 10);
    Point ringRectMR = new Point(60, 90);
    Point sampleRectBL = new Point(50, 90);
    Point sampleRectUR = new Point(90, 170);
    OpenCvInternalCamera cam;
    Servo flipper;
    DcMotorEx shootLeft;
    DcMotorEx shootRight;
    DcMotorEx intake;
    MecanumDriveRR drive;
    Pose2d initPos = new Pose2d(0, 0, 0);
    static Pose2d endPos = new Pose2d(0, 0, 0);
    OpenCvPipeline pipeline = new OpenCvPipeline() {
        @Override
        public Mat processFrame(Mat input) {
            double[] avgLowerRingRectColor = new double[3];
            double[] avgUpperRingRectColor = new double[3];
            double[] avgSampleRectColor = new double[3];
            double magLowerRingRectColor;
            double magUpperRingRectColor;
            double magSampleRectColor;
            for (int i = (int)ringRectBL.x; i <= ringRectMR.x; i++) {
                for (int j = (int)ringRectBL.y; j < ringRectMR.y; j++) {
                    avgLowerRingRectColor[0] += input.get(j, i)[0];
                    avgLowerRingRectColor[1] += input.get(j, i)[1];
                    avgLowerRingRectColor[2] += input.get(j, i)[2];
                }
            }
            for (int i = (int)ringRectML.x; i <= ringRectUR.x; i++) {
                for (int j = (int)ringRectML.y; j < ringRectUR.y; j++) {
                    avgUpperRingRectColor[0] += input.get(j, i)[0];
                    avgUpperRingRectColor[1] += input.get(j, i)[1];
                    avgUpperRingRectColor[2] += input.get(j, i)[2];
                }
            }
            for (int i = (int)sampleRectBL.x; i <= sampleRectUR.x; i++) {
                for (int j = (int)sampleRectBL.y; j <= sampleRectUR.y; j++) {
                    avgSampleRectColor[0] += input.get(j, i)[0];
                    avgSampleRectColor[1] += input.get(j, i)[1];
                    avgSampleRectColor[2] += input.get(j, i)[2];
                }
            }
            avgLowerRingRectColor[0] /= (ringRectMR.x - ringRectBL.x) * (ringRectMR.y - ringRectBL.y);
            avgLowerRingRectColor[1] /= (ringRectMR.x - ringRectBL.x) * (ringRectMR.y - ringRectBL.y);
            avgLowerRingRectColor[2] /= (ringRectMR.x - ringRectBL.x) * (ringRectMR.y - ringRectBL.y);
            magLowerRingRectColor = sqrt(pow(avgLowerRingRectColor[0], 2) + pow(avgLowerRingRectColor[1], 2) + pow(avgLowerRingRectColor[2], 2));
            avgUpperRingRectColor[0] /= (ringRectUR.x - ringRectML.x) * (ringRectUR.y - ringRectML.y);
            avgUpperRingRectColor[1] /= (ringRectUR.x - ringRectML.x) * (ringRectUR.y - ringRectML.y);
            avgUpperRingRectColor[2] /= (ringRectUR.x - ringRectML.x) * (ringRectUR.y - ringRectML.y);
            magUpperRingRectColor = sqrt(pow(avgUpperRingRectColor[0], 2) + pow(avgUpperRingRectColor[1], 2) + pow(avgUpperRingRectColor[2], 2));
            avgSampleRectColor[0] /= (sampleRectUR.x - sampleRectBL.x) * (sampleRectUR.y - sampleRectBL.y);
            avgSampleRectColor[1] /= (sampleRectUR.x - sampleRectBL.x) * (sampleRectUR.y - sampleRectBL.y);
            avgSampleRectColor[2] /= (sampleRectUR.x - sampleRectBL.x) * (sampleRectUR.y - sampleRectBL.y);
            magSampleRectColor = sqrt(pow(avgSampleRectColor[0], 2) + pow(avgSampleRectColor[1], 2) + pow(avgSampleRectColor[2], 2));
            if (avgUpperRingRectColor[0] * magSampleRectColor / (avgSampleRectColor[0] * magUpperRingRectColor) > ringRedThreshold) {
                ringNum = 4;
                telemetry.addData("Rings", 4);
            } else if (avgLowerRingRectColor[0] * magSampleRectColor / (avgSampleRectColor[0] * magLowerRingRectColor) > ringRedThreshold) {
                ringNum = 1;
                telemetry.addData("Rings", 1);
            } else {
                ringNum = 0;
                telemetry.addData("Rings", 0);
            }
            telemetry.update();
            Imgproc.rectangle(input, ringRectBL, ringRectMR, new Scalar(avgLowerRingRectColor), -1);
            Imgproc.rectangle(input, ringRectML, ringRectUR, new Scalar(avgUpperRingRectColor), -1);
            Imgproc.rectangle(input, sampleRectBL, sampleRectUR, new Scalar(avgSampleRectColor), -1);
            return input;
        }
    };

    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {

    }

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        cam.setPipeline(pipeline);
        cam.openCameraDevice();
        cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        cam.setFlashlightEnabled(true);
        drive = new MecanumDriveRR(hardwareMap);
        flipper = hardwareMap.get(Servo.class, "flipper");
        shootLeft = hardwareMap.get(DcMotorEx.class, "shootLeft");
        shootRight = hardwareMap.get(DcMotorEx.class, "shootRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shootLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPoseEstimate(initPos);
        waitForStart();
        ringNum = 4;
        if (ringNum == 4) {
            Pose2d lastPos;
            Trajectory traj1 = drive.trajectoryBuilder(initPos,0).back(24).build();
            drive.followTrajectory(traj1);
            lastPos = drive.getPoseEstimate();
            sleep(500);

            Trajectory traj2 = drive.trajectoryBuilder(lastPos,0).forward(24).build();
            drive.followTrajectory(traj2);
            lastPos = drive.getPoseEstimate();
            sleep(500);

            Trajectory traj3 = drive.trajectoryBuilder(lastPos,0).forward(24).build();
            drive.followTrajectory(traj3);
            lastPos = drive.getPoseEstimate();
            sleep(500);

            Trajectory traj4 = drive.trajectoryBuilder(lastPos,0).strafeLeft(24).build();
            drive.followTrajectory(traj4);
            lastPos = drive.getPoseEstimate();
            sleep(500);

            Trajectory traj5 = drive.trajectoryBuilder(lastPos,0).strafeRight(24).build();
            drive.followTrajectory(traj5);
            sleep(500);

            drive.turn(Math.toRadians(90));
            sleep(2500);
            drive.turn(Math.toRadians(-90));


        }
    }
}