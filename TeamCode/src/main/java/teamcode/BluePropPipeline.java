package teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BluePropPipeline extends OpenCvPipeline {

    private Mat workingMat = new Mat();

    public int position;

    public BluePropPipeline() {

    }
    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(workingMat);

        if (workingMat.empty()) {
            return input;
        }

        Imgproc.cvtColor(workingMat,workingMat,Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar (170,60,40);
        Scalar highHSV = new Scalar (230,100,100);

        Core.inRange(workingMat, lowHSV, highHSV, workingMat);

        Mat matLeft = workingMat.submat(0,720, 0,300);
        Mat matCenter = workingMat.submat(0,720,300,980);
        Mat matRight = workingMat.submat(0,720,980,1280);

        Imgproc.rectangle(workingMat, new Rect(0,0,300,720), new Scalar (0,255,0));
        Imgproc.rectangle(workingMat, new Rect(300,0,680,720), new Scalar (0,255,0));
        Imgproc.rectangle(workingMat, new Rect(980,0,300,720), new Scalar (0,255,0));

        double leftValue = Core.sumElems (matLeft).val[0];
        double centerValue = Core.sumElems (matCenter).val[0];
        double rightValue = Core.sumElems (matRight).val[0];

        matLeft.release();
        matCenter.release();
        matRight.release();

        if (leftValue > centerValue && leftValue > rightValue) {
            position = 1;
        }
        else if (centerValue > leftValue && centerValue > rightValue) {
            position = 2;
        }
        else if (rightValue > centerValue && rightValue > leftValue) {
            position = 3;
        }

        //logic

        return workingMat;

    }

    int getAnalysis() {
        return position;
    }

}
