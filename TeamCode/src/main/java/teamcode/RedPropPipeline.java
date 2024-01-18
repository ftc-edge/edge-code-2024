package teamcode;

import static org.opencv.core.Core.sumElems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class RedPropPipeline extends OpenCvPipeline {
    double location = 0;
    Mat mat;
    Mat matA;
    Mat matB;
    Telemetry telemetry;
    final Rect LEFT_ROI = new Rect(
            new Point(0, 0),
            new Point(280, 720));

    final Rect MIDDLE_ROI = new Rect(
            new Point(280, 0),
            new Point(1000, 720));

    final Rect RIGHT_ROI = new Rect(
            new Point(1000, 0),
            new Point(1280, 720));

    Scalar blue = new Scalar(0, 0, 200);
    Mat leftA;
    Mat leftB;
    Mat rightA;
    Mat rightB;
    Mat middleA;
    Mat middleB;
    Scalar lowHSV = new Scalar(0, 70, 70);
    Scalar highHSV = new Scalar(20, 255, 255);

    Scalar HlowHSV = new Scalar(160, 70, 70);
    Scalar HhighHSV = new Scalar(180, 255, 255);
    double leftValue;
    double rightValue;
    double middleValue;

    public RedPropPipeline(Telemetry telemetry1) {
        telemetry = telemetry1;
    }

    @Override
    public void init(Mat firstFrame)
    {
        mat = firstFrame;
        matA = new Mat();
        matB = new Mat();

    }

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        matA = mat;
        matB = mat;

        Core.inRange(matA, lowHSV, highHSV, matA);
        Core.inRange(matB, HlowHSV, HhighHSV, matB);

        leftA = matA.submat(LEFT_ROI);
        leftB = matB.submat(LEFT_ROI);
        rightA = matA.submat(RIGHT_ROI);
        rightB = matB.submat(RIGHT_ROI);
        middleA = matA.submat(MIDDLE_ROI);
        middleB = matB.submat(MIDDLE_ROI);

        leftValue = Core.sumElems(leftA).val[0] + Core.sumElems(leftB).val[0];
        rightValue = Core.sumElems(rightA).val[0] + Core.sumElems(rightB).val[0];
        middleValue = Core.sumElems(middleA).val[0] + Core.sumElems(middleB).val[0];

        telemetry.addData("Left raw value", leftValue);
        telemetry.addData("Right raw value", rightValue);
        telemetry.addData("Middle raw value", middleValue);

        if ((leftValue > rightValue) && (leftValue > middleValue)) {
            location = 1;
        }
        else if ((middleValue > leftValue) && (middleValue > rightValue)) {
            location = 2;
        }
        else {
            location = 3;
        }

        

        Imgproc.cvtColor(matA, matA, Imgproc.COLOR_GRAY2RGB);

        Imgproc.rectangle(matA, LEFT_ROI, blue);
        Imgproc.rectangle(matA, RIGHT_ROI, blue);
        Imgproc.rectangle(matA, MIDDLE_ROI, blue);

        leftA.release();
        leftB.release();
        rightA.release();
        rightB.release();
        middleA.release();
        middleB.release();
        mat.release();
        matB.release();


        return matA;
    }


    public double getLocation() {
        return location;
    }
}