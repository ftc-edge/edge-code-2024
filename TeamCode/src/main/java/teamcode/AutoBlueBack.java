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

import teamcode.drive.DriveConstants;
import teamcode.drive.SampleMecanumDrive;
import teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoBlueBack")
public class AutoBlueBack extends LinearOpMode {
    BluePropPipeline propPipeline;
    OpenCvCamera camera;
    double route;

    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;
    private DcMotor intake = null;
    private Servo swerver = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;

    enum State {
        TRAJECTORY_1,   // left
        TRAJECTORY_1C,
        TRAJECTORY_2,   // middle
        TRAJECTORY_2C,
        TRAJECTORY_3,   // right
        TRAJECTORY_3C,
        SLIDE_UP,       //bring up claw
        CLAW_OPEN,      // open claw
        IDLE            // Our bot will enter the IDLE state when done
    }

    @Override
    public void runOpMode() throws InterruptedException {

        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        intake = hardwareMap.get(DcMotor.class, "intake");

        swerver = hardwareMap.get(Servo.class, "swerver");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "webcam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam1, cameraMonitorViewId);

        propPipeline = new BluePropPipeline(telemetry);

        camera.setPipeline(propPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // error code
            }
        });


        Pose2d startPose = new Pose2d(13, 67.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(9, 35.00), Math.toRadians(180.00),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();



        TrajectorySequence traj3c = drive.trajectorySequenceBuilder(traj3.end())
                .lineToConstantHeading(new Vector2d(48.50, 30.00))
                .build();




        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(16.00, 40.00))
                .build();

        TrajectorySequence traj2c = drive.trajectorySequenceBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(16.00, 48.00))
                .splineTo(new Vector2d(48.50, 36.00), Math.toRadians(0.00))
                .build();



        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(24.00, 48.00), Math.toRadians(270.00))
                .build();

        TrajectorySequence traj1c = drive.trajectorySequenceBuilder(traj1.end())
                .back(10)
                .lineToLinearHeading(new Pose2d(48.50, 42.00, Math.toRadians(0.00)))
                .build();



        waitForStart();

        leftClaw.setPosition(0.45);
        rightClaw.setPosition(0.465);
        swerver.setPosition(0.6385);

        State currentState = State.IDLE;

        route = propPipeline.getLocation();

        if (route == 1) {
            currentState = State.TRAJECTORY_1;
        } else if (route == 2) {
            currentState = State.TRAJECTORY_2;
        } else if (route == 3) {
            currentState = State.TRAJECTORY_3;
        }

        camera.stopStreaming();

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(traj1);
                        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        intake.setTargetPosition(50);
                        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intake.setPower(0.05);
                        while (intake.isBusy()) {}
                        currentState = State.TRAJECTORY_1C;
                    }
                    break;

                case TRAJECTORY_2:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(traj2);
                        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        intake.setTargetPosition(50);
                        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intake.setPower(0.05);
                        while (intake.isBusy()) {}
                        currentState = State.TRAJECTORY_2C;
                    }
                    break;

                case TRAJECTORY_3:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(traj3);
                        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        intake.setTargetPosition(50);
                        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intake.setPower(0.05);
                        while (intake.isBusy()) {}
                        currentState = State.TRAJECTORY_3C;
                    }
                    break;

                case TRAJECTORY_1C:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(traj1c);
                        currentState = State.IDLE;
                    }
                    break;

                case TRAJECTORY_2C:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(traj2c);
                        currentState = State.IDLE;
                    }
                    break;

                case TRAJECTORY_3C:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(traj3c);
                        currentState = State.IDLE;
                    }
                    break;

                case SLIDE_UP:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (!drive.isBusy()) {
                        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        leftSlide.setTargetPosition(-100);
                        rightSlide.setTargetPosition(-100);
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(-0.5);
                        rightSlide.setPower(-0.5);
                        while (leftSlide.isBusy() || rightSlide.isBusy()) {}
                        swerver.setPosition(0.7);
                        currentState = State.CLAW_OPEN;
                    }
                    break;

                case CLAW_OPEN:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state

                    leftClaw.setPosition(0.485);
                    rightClaw.setPosition(0.43);
                    currentState = State.IDLE;

                    break;

                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }
        }
    }
}