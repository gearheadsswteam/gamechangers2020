package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;


public class BlueRingDetectorOpenCV extends OpenCvPipeline implements RingDetector{
    private double ringRedThreshold = 1.1;
    private int ringNum;
    private Point ringRectBL = new Point(50, 230);
    private Point ringRectUR = new Point(80, 310);
    private Point ringRectML = new Point(70, 230);
    private Point ringRectMR = new Point(60, 310);
    private Point sampleRectBL = new Point(50, 180);
    private Point sampleRectUR = new Point(80, 230);
    private OpenCvInternalCamera cam;

    private LinearOpMode curOpMode = null;   //current opmode

    /* local OpMode members. */
    private HardwareMap hardwareMap = null;

    public BlueRingDetectorOpenCV(LinearOpMode currrentOpMode, HardwareMap hwMap) {
        this.curOpMode = currrentOpMode;
        this.hardwareMap = hwMap;
    }

    @Override
    public void initialize() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        cam.setPipeline(this);
        cam.openCameraDevice();
        cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        cam.setFlashlightEnabled(false);//Setting flash light off for blue
    }

    public int detectRings() {
        return ringNum;
    }

    @Override
    public Mat processFrame(Mat input) {
        double[] avgLowerRingRectColor = new double[3];
        double[] avgUpperRingRectColor = new double[3];
        double[] avgSampleRectColor = new double[3];
        double magLowerRingRectColor;
        double magUpperRingRectColor;
        double magSampleRectColor;
        for (int i = (int) ringRectBL.x; i <= ringRectMR.x; i++) {
            for (int j = (int) ringRectBL.y; j < ringRectMR.y; j++) {
                avgLowerRingRectColor[0] += input.get(j, i)[0];
                avgLowerRingRectColor[1] += input.get(j, i)[1];
                avgLowerRingRectColor[2] += input.get(j, i)[2];
            }
        }
        for (int i = (int) ringRectML.x; i <= ringRectUR.x; i++) {
            for (int j = (int) ringRectML.y; j < ringRectUR.y; j++) {
                avgUpperRingRectColor[0] += input.get(j, i)[0];
                avgUpperRingRectColor[1] += input.get(j, i)[1];
                avgUpperRingRectColor[2] += input.get(j, i)[2];
            }
        }
        for (int i = (int) sampleRectBL.x; i <= sampleRectUR.x; i++) {
            for (int j = (int) sampleRectBL.y; j <= sampleRectUR.y; j++) {
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
        } else if (avgLowerRingRectColor[0] * magSampleRectColor / (avgSampleRectColor[0] * magLowerRingRectColor) > ringRedThreshold) {
            ringNum = 1;
        } else {
            ringNum = 0;
        }
        Imgproc.rectangle(input, ringRectBL, ringRectMR, new Scalar(avgLowerRingRectColor), -1);
        Imgproc.rectangle(input, ringRectML, ringRectUR, new Scalar(avgUpperRingRectColor), -1);
        Imgproc.rectangle(input, sampleRectBL, sampleRectUR, new Scalar(avgSampleRectColor), -1);
        return input;
    }
}


