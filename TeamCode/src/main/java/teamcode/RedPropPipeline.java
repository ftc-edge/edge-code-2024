package teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class RedPropPipeline extends OpenCvPipeline {

    private Mat workingMat = new Mat();

    public int position;

    public RedPropPipeline() {

    }
    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(workingMat);

        if (workingMat.empty()) {
            return input;
        }

        Imgproc.cvtColor(workingMat,workingMat,Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar (330,70,70);
        Scalar highHSV = new Scalar (360,100,100);

        Core.inRange(workingMat, lowHSV, highHSV, workingMat);

        Mat matLeft = workingMat.submat(360,720, 0,430);
        Mat matCenter = workingMat.submat(360,720,430,850);
        Mat matRight = workingMat.submat(360,720,850,1280);

        Imgproc.rectangle(workingMat, new Rect(0,360,430,360), new Scalar (0,255,0));
        Imgproc.rectangle(workingMat, new Rect(430,360,420,360), new Scalar (0,255,0));
        Imgproc.rectangle(workingMat, new Rect(850,360,430,360), new Scalar (0,255,0));

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