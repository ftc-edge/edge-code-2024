/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package multiteams;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Rect;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcTensorFlow;
import TrcFtcLib.ftclib.FtcVuforia;

/**
 * This class implements Vuforia/TensorFlow/Grip/Eocv Vision for the game season. It creates and initializes all the
 * vision target info as well as providing info for the robot, camera and the field. It also provides methods to get
 * the location of the robot and detected targets.
 */
public class Vision
{
    public static final String OPENCV_NATIVE_LIBRARY_NAME = "EasyOpenCV";
    public static final String[] TARGET_LABELS = {
        BlinkinLEDs.LABEL_TARGET1, BlinkinLEDs.LABEL_TARGET2, BlinkinLEDs.LABEL_TARGET3};

    private final Robot robot;
    public VuforiaVision vuforiaVision;
    public TensorFlowVision tensorFlowVision;
    public EocvVision eocvVision;
    private int aprilTagId = 0;

    /**
     * Constructor: Create an instance of the object. Vision is required by both Vuforia and TensorFlow and must be
     * instantiated if either is used. However, to use either Vuforia or TensorFlow, one must explicitly initialize
     * them by calling the initVuforia or initTensorFlow methods respectively.
     *
     * @param robot specifies the robot object.
     */
    public Vision(Robot robot)
    {
        FtcOpMode opMode = FtcOpMode.getInstance();
        int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        this.robot = robot;
        if (RobotParams.Preferences.useEasyOpenCV)
        {
            OpenCvWebcam webcam;

            if (RobotParams.Preferences.showEasyOpenCvView)
            {
                webcam = OpenCvCameraFactory.getInstance().createWebcam(
                    opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM), cameraViewId);
                webcam.showFpsMeterOnViewport(false);
            }
            else
            {
                webcam = OpenCvCameraFactory.getInstance().createWebcam(
                    opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM));
            }
            webcam.setMillisecondsPermissionTimeout(RobotParams.WEBCAM_PERMISSION_TIMEOUT);
            eocvVision = new EocvVision(
                "EocvVision", RobotParams.CAMERA_IMAGE_WIDTH, RobotParams.CAMERA_IMAGE_HEIGHT,
                RobotParams.cameraRect, RobotParams.worldRect, webcam, OpenCvCameraRotation.UPRIGHT, null);
        }
        else if (RobotParams.Preferences.useVuforia || RobotParams.Preferences.useTensorFlow)
        {
            final String VUFORIA_LICENSE_KEY =
                "ARbBwjf/////AAABmZijKPKUWEY+uNSzCuTOUFgm7Gr5irDO55gtIOjsOXmhLzLEILJp45qdPrwMfoBV2Yh7F+Wh8iEjnSA" +
                "NnnRKiJNHy1T9Pr2uufETE40YJth10Twv0sTNSEqxDPhg2t4PJXwRImMaEsTE53fmcm08jT9qMso2+1h9eNk2b4x6DVKgBt" +
                "Tv5wocDs949Gkh6lRt5rAxATYYO9esmyKyfyzfFLMMpfq7/uvQQrSibNBqa13hJRmmHoM2v0Gfk8TCTTfP044/XsOm54u8k" +
                "dv0HfeMBC91uQ/NvWHVV5XCh8pZAzmL5sry1YwG8FSRNVlSAZ1zN/m6jAe98q6IxpwQxP0da/TpJoqDI7x4RGjOs1Areunf";
            //
            // If no camera view ID, do not activate camera monitor view to save power.
            //
            VuforiaLocalizer.Parameters vuforiaParams =
                RobotParams.Preferences.showVuforiaView?
                    new VuforiaLocalizer.Parameters(cameraViewId): new VuforiaLocalizer.Parameters();

            vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
            vuforiaParams.cameraName = opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM);
            vuforiaParams.useExtendedTracking = false;
            vuforiaParams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
            FtcVuforia vuforia = new FtcVuforia(vuforiaParams);

            vuforiaVision = RobotParams.Preferences.useVuforia? new VuforiaVision(vuforia, robot.blinkin): null;
            tensorFlowVision = RobotParams.Preferences.useTensorFlow? new TensorFlowVision(vuforia, null): null;
        }
    }   //Vision

    /**
     * This method shuts down TensorFlow.
     */
    public void tensorFlowShutdown()
    {
        if (tensorFlowVision != null)
        {
            tensorFlowVision.shutdown();
            tensorFlowVision = null;
        }
    }   //tensorFlowShutdown

    /**
     * This method updates the LED state to show the vision detected object.
     *
     * @param label specifies the detected object.
     */
    private void updateVisionLEDs(String label)
    {
        if (label != null && robot.blinkin != null)
        {
            robot.blinkin.setPatternState(BlinkinLEDs.LABEL_TARGET1, false);
            robot.blinkin.setPatternState(BlinkinLEDs.LABEL_TARGET2, false);
            robot.blinkin.setPatternState(BlinkinLEDs.LABEL_TARGET3, false);
            robot.blinkin.setPatternState(label, true, 1.0);
        }
    }   //updateVisionLEDs

    /**
     * This method calls TensorFlow vision to detect the object and to return the detected info.
     *
     * @return detected object info, null if none detected.
     */
    public TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> getDetectedTensorFlowInfo()
    {
        TrcVisionTargetInfo<FtcTensorFlow.DetectedObject>[] targets = null;
        String label = null;

        if (tensorFlowVision != null && tensorFlowVision.isEnabled())
        {
            targets = tensorFlowVision.getDetectedTargetsInfo(
                null, null, this::compareConfidence, RobotParams.TAG_HEIGHT_OFFSET, RobotParams.cameraHeightOffset);
            label = targets[0].detectedObj.label;
        }
        updateVisionLEDs(label);

        return targets != null? targets[0]: null;
    }   //getDetectedTensorFlowInfo

    /**
     * This method calls vision to detect the AprilTag and to return the detected info.
     *
     * @return detected AprilTag info, null if none detected.
     */
    public TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> getDetectedAprilTagInfo()
    {
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>>[] targets = null;
        String label = null;

        if (eocvVision != null && eocvVision.isEnabled())
        {
            int id = 0;

            eocvVision.setDetectObjectType(EocvVision.ObjectType.APRIL_TAG);
            targets = eocvVision.getDetectedTargetsInfo(
                null, null, RobotParams.TAG_HEIGHT_OFFSET, RobotParams.cameraHeightOffset);
            if (targets != null)
            {
                id = ((AprilTagDetection) targets[0].detectedObj.object).id;
            }

            switch (id)
            {
                case 1:
                    label = BlinkinLEDs.LABEL_TARGET1;
                    break;

                case 2:
                    label = BlinkinLEDs.LABEL_TARGET2;
                    break;

                case 3:
                    label = BlinkinLEDs.LABEL_TARGET3;
                    break;
            }

            if (id != 0)
            {
                aprilTagId = id;
            }
        }
        updateVisionLEDs(label);

        return targets != null? targets[0]: null;
    }   //getDetectedAprilTagInfo

    /**
     * This method returns the last detected AprilTag ID.
     *
     * @return last detected AprilTag ID.
     */
    public int getAprilTagId()
    {
        return aprilTagId;
    }   //getAprilTagId

    /**
     * This method calls vision to detect the color blob and returns the detected info.
     *
     * @return detected color blob info, null if none detected.
     */
    @SuppressWarnings("unchecked")
    public TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> getDetectedColorBlobInfo(
        EocvVision.ObjectType objectType)
    {
        TrcVisionTargetInfo<?>[] targets = null;

        if (eocvVision != null && eocvVision.isEnabled())
        {
            eocvVision.setDetectObjectType(objectType);
            targets = eocvVision.getDetectedTargetsInfo(null, this::compareBottomY, 0.0, 0.0);
            if (targets != null && robot.blinkin != null)
            {
                robot.blinkin.setPatternState(BlinkinLEDs.DETECTED_RED_BLOB, false);
                robot.blinkin.setPatternState(BlinkinLEDs.DETECTED_BLUE_BLOB, false);
                robot.blinkin.setPatternState(BlinkinLEDs.DETECTED_YELLOW_BLOB, false);
                switch (objectType)
                {
                    case RED_BLOB:
                        robot.blinkin.setPatternState(BlinkinLEDs.DETECTED_RED_BLOB, true, 1.0);
                        break;

                    case BLUE_BLOB:
                        robot.blinkin.setPatternState(BlinkinLEDs.DETECTED_BLUE_BLOB, true, 1.0);
                        break;

                    case YELLOW_BLOB:
                        robot.blinkin.setPatternState(BlinkinLEDs.DETECTED_YELLOW_BLOB, true, 1.0);
                        break;
                }
            }
        }

        return targets != null? (TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>) targets[0]: null;
    }   //getDetectedColorBlobInfo

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing confidence.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has higher confidence than b, 0 if a and b have equal confidence, positive value
     *         if a has lower confidence than b.
     */
    private int compareConfidence(
        TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> a, TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> b)
    {
        return (int)((b.detectedObj.confidence - a.detectedObj.confidence)*100);
    }   //compareConfidence

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing bottom Y.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller bottom Y than b, 0 if a and b have equal bottom Y,
     *         positive value if a has larger bottom Y than b.
     */
    private int compareBottomY(
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> a,
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> b)
    {
        Rect aRect = a.detectedObj.getRect();
        Rect bRect = b.detectedObj.getRect();

        return (bRect.y + bRect.height) - (aRect.y + aRect.height);
    }   //compareBottomY

}   //class Vision
