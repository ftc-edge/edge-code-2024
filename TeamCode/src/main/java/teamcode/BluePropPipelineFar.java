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

class BluePropPipelineFar extends OpenCvPipeline {
    double location = 0;
    Mat mat;
    Telemetry telemetry;
    Rect LEFT_ROI = new Rect(
            new Point(0, 200),
            new Point(300, 720));

    Rect MIDDLE_ROI = new Rect(
            new Point(300, 200),
            new Point(750, 720));

    Rect RIGHT_ROI = new Rect(
            new Point(750, 200),
            new Point(1280, 720));
    Scalar blue = new Scalar(0, 0, 200);
    Mat left;
    Mat right;
    Mat middle;
    Scalar lowHSV = new Scalar(100, 160, 120);
    Scalar highHSV = new Scalar(140, 255, 255);
    double leftValue;
    double rightValue;
    double middleValue;

    public BluePropPipelineFar(Telemetry telemetry1) {
        telemetry = telemetry1;
    }

    @Override
    public void init(Mat firstFrame)
    {
        mat = firstFrame;

    }


    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(mat, lowHSV, highHSV, mat);

        left = mat.submat(LEFT_ROI);
        right = mat.submat(RIGHT_ROI);
        middle = mat.submat(MIDDLE_ROI);

        leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area();
        rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area();
        middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area();

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


        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Imgproc.rectangle(mat, LEFT_ROI, blue);
        Imgproc.rectangle(mat, RIGHT_ROI, blue);
        Imgproc.rectangle(mat, MIDDLE_ROI, blue);

        left.release();
        right.release();
        middle.release();

        return mat;
    }


    public double getLocation() {
        return location;
    }
}