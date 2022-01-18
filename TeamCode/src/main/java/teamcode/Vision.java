/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
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

package teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Point;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcHashMap;
import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRevBlinkin;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcTensorFlow;
import TrcFtcLib.ftclib.FtcVuforia;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import java.util.Comparator;

/**
 * This class implements Vuforia/TensorFlow Vision for the game season. It creates and initializes all the vision
 * target info as well as providing info for the robot, camera and the field. It also provides methods to get the
 * location of the robot and detected targets.
 */
public class Vision
{
    public static final String IMAGE_1 = "Image1";
    public static final String IMAGE_2 = "Image2";
    public static final String IMAGE_3 = "Image3";
    public static final String IMAGE_4 = "Image4";
    public static final String LABEL_1 = "Object1";
    public static final String LABEL_2 = "Object2";
    public static final String LABEL_3 = "Object3";
    private final TrcHashMap<String, TrcRevBlinkin.LEDPattern> targetLEDPatternMap =
        new TrcHashMap<String, TrcRevBlinkin.LEDPattern>()
            .add(LABEL_1, TrcRevBlinkin.LEDPattern.SolidRed)
            .add(LABEL_2, TrcRevBlinkin.LEDPattern.SolidGreen)
            .add(LABEL_3, TrcRevBlinkin.LEDPattern.SolidBlue)
            .add(IMAGE_1, TrcRevBlinkin.LEDPattern.FixedStrobeRed)
            .add(IMAGE_2, TrcRevBlinkin.LEDPattern.FixedStrobeBlue)
            .add(IMAGE_3, TrcRevBlinkin.LEDPattern.FixedLightChaseRed)
            .add(IMAGE_4, TrcRevBlinkin.LEDPattern.FixedLightChaseBlue);
    private final TrcRevBlinkin.LEDPattern[] ledPatternPriorities = {
        TrcRevBlinkin.LEDPattern.SolidRed,
        TrcRevBlinkin.LEDPattern.SolidGreen,
        TrcRevBlinkin.LEDPattern.SolidBlue,
        TrcRevBlinkin.LEDPattern.FixedStrobeRed,
        TrcRevBlinkin.LEDPattern.FixedStrobeBlue,
        TrcRevBlinkin.LEDPattern.FixedLightChaseRed,
        TrcRevBlinkin.LEDPattern.FixedLightChaseBlue};

    private final Robot robot;
    private final TrcDbgTrace tracer;
    private final FtcVuforia vuforia;
    //
    // Vuforia Vision.
    //
    private final VuforiaVision vuforiaVision;
    private String lastVuforiaImageName;
    //
    // TensorFlow Vision.
    //
    private TensorFlowVision tensorFlowVision;

    /**
     * Constructor: Create an instance of the object. Vision is required by both Vuforia and TensorFlow and must be
     * instantiated if either is used. However, to use either Vuforia or TensorFlow, one must explicitly initialize
     * them by calling the initVuforia or initTensorFlow methods respectively.
     *
     * @param robot specifies the robot object.
     * @param useVuforia specifies true to use Vuforia Vision, false otherwise.
     * @param useTensorFlow specifies true to use TensorFlow Vision, false otherwise.
     */
    public Vision(Robot robot, boolean useVuforia, boolean useTensorFlow)
    {
        this.robot = robot;
        this.tracer = TrcDbgTrace.getGlobalTracer();
        final String VUFORIA_LICENSE_KEY =
            "ARbBwjf/////AAABmZijKPKUWEY+uNSzCuTOUFgm7Gr5irDO55gtIOjsOXmhLzLEILJp45qdPrwMfoBV2Yh7F+Wh8iEjnSA" +
            "NnnRKiJNHy1T9Pr2uufETE40YJth10Twv0sTNSEqxDPhg2t4PJXwRImMaEsTE53fmcm08jT9qMso2+1h9eNk2b4x6DVKgBt" +
            "Tv5wocDs949Gkh6lRt5rAxATYYO9esmyKyfyzfFLMMpfq7/uvQQrSibNBqa13hJRmmHoM2v0Gfk8TCTTfP044/XsOm54u8k" +
            "dv0HfeMBC91uQ/NvWHVV5XCh8pZAzmL5sry1YwG8FSRNVlSAZ1zN/m6jAe98q6IxpwQxP0da/TpJoqDI7x4RGjOs1Areunf";
        FtcOpMode opMode = FtcOpMode.getInstance();
        int cameraViewId = !RobotParams.Preferences.showVuforiaView ? -1 :
            opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        //
        // If no camera view ID, do not activate camera monitor view to save power.
        //
        VuforiaLocalizer.Parameters vuforiaParams =
            cameraViewId == -1? new VuforiaLocalizer.Parameters(): new VuforiaLocalizer.Parameters(cameraViewId);

        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraName = opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM);
        vuforiaParams.useExtendedTracking = false;
        vuforiaParams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = new FtcVuforia(vuforiaParams);

        vuforiaVision = useVuforia? new VuforiaVision(): null;
        tensorFlowVision = useTensorFlow? new TensorFlowVision(): null;
    }   //Vision

    /**
     * This method sets up the Blinkin with a priority pattern list and a pattern name map.
     */
    public void setupBlinkin()
    {
        robot.blinkin.setNamedPatternMap(targetLEDPatternMap);
        robot.blinkin.setPatternPriorities(ledPatternPriorities);
    }   //setupBlinkin

    /**
     * This method checks if Vuforia vision is initialized.
     *
     * @return true if Vuforia vision is initialized, false otherwise.
     */
    public boolean isVuforiaVisionInitialized()
    {
        return vuforiaVision != null;
    }   //isVuforiaVisionInitialized

    /**
     * This method enables/disables Vuforia Vision.
     *
     * @param enabled specifies true to enable Vuforia Vision, false to disable.
     */
    public void setVuforiaEnabled(boolean enabled)
    {
        if (vuforiaVision == null) throw new RuntimeException("Vuforia Vision is not initialized.");

        vuforia.setTrackingEnabled(enabled);
    }   //setVuforiaEnabled

    /**
     * This method returns the name of the image last seen.
     *
     * @return last seen image name.
     */
    public String getLastSeenVuforiaImageName()
    {
        return lastVuforiaImageName;
    }   //getLastSeenVuforiaImageName

    /**
     * This method returns the robot field position.
     *
     * @param targetName specifies the detected target name.
     * @param exclude specifies true to exclude the specified target.
     * @return robot field position.
     */
    public TrcPose2D getRobotPose(String targetName, boolean exclude)
    {
        if (vuforiaVision == null) throw new RuntimeException("Vuforia Vision is not initialized.");

        OpenGLMatrix robotLocation = vuforiaVision.getRobotLocation(targetName, exclude);
        VectorF translation = robotLocation == null? null: vuforiaVision.getLocationTranslation(robotLocation);
        Orientation orientation = robotLocation == null? null: vuforiaVision.getLocationOrientation(robotLocation);
        //
        // The returned RobotPose have the X axis pointing from the audience side to the back of the field,
        // the Y axis pointing from the red alliance to the blue alliance and the direction of the Y axis
        // is zero degree and increases in the clockwise direction.
        // Vuforia's orientation is such that facing the negative side of the X axis is 0 degree, clockwise gives
        // you negative angle and anti-clockwise gives you positive angle.
        // Robot's orientation is such that facing the positive side of the Y axis is 0 degree, clockwise gives
        // you positive angle and anti-clockwise gives you negative angle.
        // In order to translate Vuforia's orientation to the robot's orientation, we first negate the vuforia
        // angle to make rotation agree with the robot's rotation. Then we subtract 90 degrees from the angle.
        //
        return (translation == null || orientation == null)? null:
            new TrcPose2D(translation.get(0)/TrcUtil.MM_PER_INCH, translation.get(1)/TrcUtil.MM_PER_INCH,
                          -orientation.thirdAngle - 90.0);
    }   //getRobotPose

    /**
     * This method checks if TensorFlow vision is initialized.
     *
     * @return true if TensorFlow vision is initialized, false otherwise.
     */
    public boolean isTensorFlowVisionInitialized()
    {
        return tensorFlowVision != null;
    }   //isTensorFlowVisionInitialized

    /**
     * This method enables/disables TensorFlow.
     *
     * @param enabled specifies true to enable TensorFlow, false to disable.
     */
    public void setTensorFlowEnabled(boolean enabled)
    {
        if (tensorFlowVision == null) throw new RuntimeException("TensorFlow Vision is not initialized!");

        tensorFlowVision.tensorFlow.setEnabled(enabled);
        if (enabled)
        {
            tensorFlowVision.tensorFlow.setZoom(1.0, 16.0/9.0);
        }
    }   //setTensorFlowEnabled

    public void setTensorFlowZoomFactor(double factor)
    {
        if (tensorFlowVision == null) throw new RuntimeException("TensorFlow Vision is not initialized!");

        tensorFlowVision.tensorFlow.setZoom(factor, 16.0/9.0);
    }   //setTensorFlowZoomFactor

    /**
     * This method shuts down TensorFlow.
     */
    public void tensorFlowShutdown()
    {
        setTensorFlowEnabled(false);
        tensorFlowVision.shutdown();
        tensorFlowVision = null;
    }   //tensorFlowShutdown

    /**
     * This method detects targets that matches the given label and filtering criteria.
     *
     * @param label specifies the target label to detect, can be null if detects any target.
     * @param filter specifies the filter method to call to filter out false positives, can be null if not provided.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     *
     * @return an array of detected targets, null if none found at the moment.
     */
    public FtcTensorFlow.TargetInfo[] getDetectedTargetsInfo(
        String label, FtcTensorFlow.FilterTarget filter, Comparator<? super FtcTensorFlow.TargetInfo> comparator)
    {
        if (tensorFlowVision == null) throw new RuntimeException("TensorFlow Vision is not initialized!");

        return tensorFlowVision.getDetectedTargetsInfo(label, filter, comparator);
    }   //getDetectedTargetsInfo

    /**
     * This method returns the best detected targets from TensorFlow vision.
     *
     * @param label specifies the label of the targets to detect for, can be null for detecting any target.
     * @param filter specifies the filter method to call to filter out false positives, can be null if not provided.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @param showAllTargets specifies true to display info for all detected targets.
     * @return the best detected target info.
     */
    public FtcTensorFlow.TargetInfo getBestDetectedTargetInfo(
        String label, FtcTensorFlow.FilterTarget filter,
        Comparator<? super FtcTensorFlow.TargetInfo> comparator, boolean showAllTargets)
    {
        FtcTensorFlow.TargetInfo[] targets = getDetectedTargetsInfo(label, filter, comparator);

        if (targets != null && showAllTargets)
        {
            for (int i = 0; i < targets.length; i++)
            {
                tracer.traceInfo(
                    "Vision.getBestDetectedTargetInfo", "[%d] Target=%s", i, targets[i]);
            }
        }

        return targets != null? targets[0]: null;
    }   //getBestDetectedTargetInfo

    /**
     * This method returns the best detected targets from TensorFlow vision.
     *
     * @param label specifies the label of the targets to detect for, can be null for detecting any target.
     * @param filter specifies the filter method to call to filter out false positives, can be null if not provided.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return the best detected target info.
     */
    public FtcTensorFlow.TargetInfo getBestDetectedTargetInfo(
        String label, FtcTensorFlow.FilterTarget filter, Comparator<? super FtcTensorFlow.TargetInfo> comparator)
    {
        return getBestDetectedTargetInfo(label, filter, comparator, false);
    }   //getBestDetectedTargetInfo

    /**
     * This class implements Vuforia Vision that provides the capability of using images to locate the robot position
     * on the field.
     */
    private class VuforiaVision
    {
        private static final int IMAGE_WIDTH = 640;     //in pixels
        private static final int IMAGE_HEIGHT = 480;    //in pixels
        private static final int FRAME_QUEUE_CAPACITY = 2;
        //
        // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical
        // dimension. We will define some constants and conversions here.
        //
        // Height of the center of the target image above the floor.
        private static final float mmTargetHeight = 6.0f * (float)TrcUtil.MM_PER_INCH;
        private static final float halfField = (float)(RobotParams.HALF_FIELD_INCHES*TrcUtil.MM_PER_INCH);
        private static final float fullTile = (float)(RobotParams.FULL_TILE_INCHES*TrcUtil.MM_PER_INCH);
        private static final float halfTile = (float)(RobotParams.HALF_TILE_INCHES*TrcUtil.MM_PER_INCH);
        private static final float oneAndHalfTile = (float)(fullTile*1.5);

        private final VuforiaTrackable[] vuforiaImageTargets;

        /**
         * Constructor: Create an instance of the object.
         */
        private VuforiaVision()
        {
            vuforia.configVideoSource(IMAGE_WIDTH, IMAGE_HEIGHT, FRAME_QUEUE_CAPACITY);
            /*
             * Create a transformation matrix describing where the camera is on the robot.
             *
             * Info:  The coordinate frame for the robot looks the same as the field.
             * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along
             * the Y axis. Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
             *
             * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z
             * direction), with the wide (horizontal) axis of the camera aligned with the X axis, and
             * the Narrow (vertical) axis of the camera aligned with the Y axis
             *
             * But, this example assumes that the camera is actually facing forward out the front of the robot.
             * So, the "default" camera position requires two rotations to get it oriented correctly.
             * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out
             *    the right side of the robot)
             * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
             *
             * Finally the camera can be translated to its actual mounting position on the robot.
             */
            final float CAMERA_FORWARD_DISPLACEMENT =
                (float)((RobotParams.ROBOT_LENGTH/2.0 - RobotParams.CAMERA_FRONT_OFFSET)*TrcUtil.MM_PER_INCH);
            final float CAMERA_VERTICAL_DISPLACEMENT =
                (float)(RobotParams.CAMERA_HEIGHT_OFFSET*TrcUtil.MM_PER_INCH);
            final float CAMERA_LEFT_DISPLACEMENT =
                (float)((RobotParams.ROBOT_WIDTH/2.0 - RobotParams.CAMERA_LEFT_OFFSET)*TrcUtil.MM_PER_INCH);
            OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, 90, 90, 0));
            /*
             * In order for localization to work, we need to tell the system where each target is on the field,
             * and where the camera resides on the robot.  These specifications are in the form of
             * <em>transformation matrices.</em>
             * Transformation matrices are a central, important concept in the math here involved in localization.
             * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
             * for detailed information. Commonly, you'll encounter transformation matrices as instances
             * of the {@link OpenGLMatrix} class.
             *
             * If you are standing in the Red Alliance Station looking towards the center of the field,
             *     - The X axis runs from your left to the right. (positive from the center to the right)
             *     - The Y axis runs from the Red Alliance Station towards the other side of the field
             *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance
             *       station)
             *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
             *
             * Before being transformed, each target image is conceptually located at the origin of the field's
             * coordinate system (the center of the field), facing up.
             */
            OpenGLMatrix image1Location = OpenGLMatrix
                .translation(-halfField, oneAndHalfTile, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
            OpenGLMatrix image2Location = OpenGLMatrix
                .translation(halfTile, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
            OpenGLMatrix image3Location = OpenGLMatrix
                .translation(-halfField, -oneAndHalfTile, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
            OpenGLMatrix image4Location = OpenGLMatrix
                .translation(halfTile, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
            //
            // Create and initialize all image targets.
            //
            FtcVuforia.TargetInfo[] imageTargetsInfo = {
                new FtcVuforia.TargetInfo(0, IMAGE_1, false, image1Location),
                new FtcVuforia.TargetInfo(1, IMAGE_2, false, image2Location),
                new FtcVuforia.TargetInfo(2, IMAGE_3, false, image3Location),
                new FtcVuforia.TargetInfo(3, IMAGE_4, false, image4Location)
            };
            vuforia.addTargetList(RobotParams.TRACKABLE_IMAGES_FILE, imageTargetsInfo, cameraLocationOnRobot);

            vuforiaImageTargets = new VuforiaTrackable[imageTargetsInfo.length];
            for (int i = 0; i < vuforiaImageTargets.length; i++)
            {
                vuforiaImageTargets[i] = vuforia.getTarget(imageTargetsInfo[i].name);
            }
        }   //VuforiaVision

        /**
         * This method returns the vector of the given target location object.
         *
         * @param location specifies the target location.
         * @return target location vector.
         */
        private VectorF getLocationTranslation(OpenGLMatrix location)
        {
            return location.getTranslation();
        }   //getLocationTranslation

        /**
         * This method returns the orientation of the given target location object.
         *
         * @param location specifies the target location.
         * @return target orientation.
         */
        private Orientation getLocationOrientation(OpenGLMatrix location)
        {
            return Orientation.getOrientation(location, EXTRINSIC, XYZ, DEGREES);
        }   //getLocationOrientation

        /**
         * This method returns the robot location computed with the detected target.
         *
         * @param targetName specifies the detected target name.
         * @return robot location.
         */
        private OpenGLMatrix getRobotLocation(String targetName)
        {
            OpenGLMatrix robotLocation = null;
            VuforiaTrackable target = vuforia.getTarget(targetName);

            if (target != null)
            {
                robotLocation = vuforia.getRobotLocation(target);

                if (robot.blinkin != null)
                {
                    if (robotLocation != null)
                    {
                        robot.blinkin.setPatternState(targetName, true);
                    }
                    else
                    {
                        robot.blinkin.reset();
                    }
                }
            }

            return robotLocation;
        }   //getRobotLocation

        /**
         * This method returns the robot location computed with the detected target.
         *
         * @param targetName specifies the detected target name.
         * @param exclude specifies true to exclude the specified target.
         * @return robot location.
         */
        private OpenGLMatrix getRobotLocation(String targetName, boolean exclude)
        {
            OpenGLMatrix robotLocation = null;

            if (targetName == null || exclude)
            {
                for (VuforiaTrackable target: vuforiaImageTargets)
                {
                    String name = target.getName();
                    boolean isMatched = targetName == null || !targetName.equals(name);

                    if (isMatched && vuforia.isTargetVisible(target))
                    {
                        // getRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix location = vuforia.getRobotLocation(target);
                        if (location != null)
                        {
                            robotLocation = location;
                            lastVuforiaImageName = name;
                        }
                        break;
                    }
                }
            }
            else
            {
                robotLocation = getRobotLocation(targetName);
                if (robotLocation != null)
                {
                    lastVuforiaImageName = targetName;
                }
            }

            if (robot.blinkin != null)
            {
                if (robotLocation != null)
                {
                    robot.blinkin.setPatternState(lastVuforiaImageName, true);
                }
                else
                {
                    robot.blinkin.reset();
                }
            }

            return robotLocation;
        }   //getRobotLocation

    }   //class VuforiaVision

    /**
     * This class implements TensorFlow Vision that provides the capability to detect learned objects and return their
     * location info.
     */
    private class TensorFlowVision
    {
        private static final String OPENCV_NATIVE_LIBRARY_NAME = "opencv_java3";
        private static final String TFOD_MODEL_ASSET = "GameModel.tflite";
        private static final float TFOD_MIN_CONFIDENCE = 0.8f;

        private FtcTensorFlow tensorFlow;

        /**
         * Constructor: Create an instance of the object.
         */
        private TensorFlowVision()
        {
            System.loadLibrary(OPENCV_NATIVE_LIBRARY_NAME);
            FtcOpMode opMode = FtcOpMode.getInstance();
            int tfodMonitorViewId = !RobotParams.Preferences.showTensorFlowView ? -1 :
                opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            //
            // If no TFOD monitor view ID, do not activate camera monitor view to save power.
            //
            TFObjectDetector.Parameters tfodParams =
                tfodMonitorViewId == -1?
                    new TFObjectDetector.Parameters() : new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParams.minResultConfidence = TFOD_MIN_CONFIDENCE;
            tfodParams.isModelTensorFlow2 = true;
            tfodParams.inputSize = 320;

            TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
                RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_Y,
                RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_Y,
                RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y,
                RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y);

            TrcHomographyMapper.Rectangle worldRect = new TrcHomographyMapper.Rectangle(
                RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_X, RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_Y,
                RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_Y,
                RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_Y,
                RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y);

            tensorFlow = new FtcTensorFlow(
                vuforia, tfodParams, TFOD_MODEL_ASSET, new String[] {LABEL_1, LABEL_2, LABEL_3}, cameraRect, worldRect,
                tracer);
        }   //TensorFlowVision

        /**
         * This method shuts down TensorFlow.
         */
        private void shutdown()
        {
            if (tensorFlow != null)
            {
                tensorFlow.shutdown();
                tensorFlow = null;
            }
        }   //shutdown

        /**
         * This method returns an array of detected targets from TensorFlow vision.
         *
         * @param label specifies the label of the targets to detect for, can be null for detecting any target.
         * @param filter specifies the filter to call to filter out false positive targets.
         * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
         * @return array of detected target info.
         */
        private FtcTensorFlow.TargetInfo[] getDetectedTargetsInfo(
            String label, FtcTensorFlow.FilterTarget filter, Comparator<? super FtcTensorFlow.TargetInfo> comparator)
        {
            return tensorFlow.getDetectedTargetsInfo(label, filter, comparator);
        }   //getDetectedTargetsInfo

        /**
         * This method maps a camera screen point to a real-world point.
         *
         * @param point specifies the camera screen point.
         * @return real-world point.
         */
        public Point mapPoint(Point point)
        {
            return tensorFlow != null? tensorFlow.mapPoint(point): null;
        }   //mapPoint

        /**
         * This method maps a camera screen point to a real-world point.
         *
         * @param x specifies the camera screen point x.
         * @param y specifies the camera screen point y.
         * @return real-world point.
         */
        public Point mapPoint(double x, double y)
        {
            return mapPoint(new Point(x, y));
        }   //mapPoint

    }   //class TensorFlowVision

}   //class Vision
