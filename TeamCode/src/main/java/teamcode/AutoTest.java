package teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import teamcode.drive.SampleMecanumDrive;
import teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="AutoTest")
public class AutoTest extends LinearOpMode {

    public static double x = 0;

    public static double y = 0;

    public static int intakePos = 0;
    private DcMotor intake = null;

    @Override
    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(16.05, -67.50, Math.toRadians(90.00)))
                .lineTo(new Vector2d(x, y))
                .build();



        waitForStart();




        drive.followTrajectorySequence(traj2);




        while(opModeIsActive() && !isStopRequested()) {
            requestOpModeStop();
        }



    }
}
