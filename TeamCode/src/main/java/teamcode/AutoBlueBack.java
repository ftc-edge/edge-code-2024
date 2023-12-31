package teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import teamcode.drive.SampleMecanumDrive;

@Autonomous(name="AutoBlueBack")
public class AutoBlueBack extends LinearOpMode {

    BluePropPipeline bluePipeline;
    OpenCvCamera camera;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "webcam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam1, cameraMonitorViewId);

        bluePipeline = new BluePropPipeline();

        camera.setPipeline(bluePipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        int route = bluePipeline.getAnalysis();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(0,48))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(48,0))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(0,-48))
                .build();

        waitForStart();

        if (route == 1) {
            drive.followTrajectory(traj1);
        }

        else if (route == 2) {
            drive.followTrajectory(traj2);
        }

        else if (route == 3) {
            drive.followTrajectory(traj3);
        }

    }
}