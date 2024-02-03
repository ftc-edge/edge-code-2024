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
    boolean extend = false;
    boolean extaking = false;
    boolean intaking = false;
    Pose2d cycleStart = new Pose2d(48.50, 35.00, Math.toRadians(180));
    double leftClawPos;
    double rightClawPos;
    double leftSwerverPos;
    double rightSwerverPos;
    double rightClawOpen = 0.5;
    double rightClawClosed = 0.7;
    double leftClawOpen = 0.47;
    double leftClawClosed = 0.27;
    double rightSwerverDropper = 0.05;
    double rightSwerverVert = 0.56;
    double rightSwerverPropel = 0.67;
    double leftSwerverDropper = 0.92;
    double leftSwerverVert = 0.41;
    double leftSwerverPropel = 0.3;


    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;
    private DcMotor intake = null;
    private Servo leftSwerver = null;
    private Servo rightSwerver = null;
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
        SWERVE_BACK,
        BOX_OPEN,      // open claw
        SWERVE_FORWARD,
        SLIDE_DOWN,
        MOVE_IN,
        CYCLE_1,
        IDLE            // Our bot will enter the IDLE state when done
    }

    @Override
    public void runOpMode() throws InterruptedException {

        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftSwerver = hardwareMap.get(Servo.class, "leftSwerver");
        rightSwerver = hardwareMap.get(Servo.class, "rightSwerver");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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


        Pose2d startPose = new Pose2d(14, 67.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(12, 32.00,Math.toRadians(180)), Math.toRadians(180.00))
                .build();



        TrajectorySequence traj3c = drive.trajectorySequenceBuilder(traj3.end())
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(52, 28.00),Math.toRadians(0))
                .build();




        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(24.00, 24.00,Math.toRadians(180)),Math.toRadians(270))
                .build();

        TrajectorySequence traj2c = drive.trajectorySequenceBuilder(traj2.end())
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(52, 35.00),Math.toRadians(0))
                .build();



        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(26.00, 40.00, Math.toRadians(270)), Math.toRadians(0.00))
                .build();

        TrajectorySequence traj1c = drive.trajectorySequenceBuilder(traj1.end())
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(52, 45.00, Math.toRadians(180.00)))
                .build();






        TrajectorySequence cycle = drive.trajectorySequenceBuilder(cycleStart)
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-58, 0), Math.toRadians(180.00))
                .splineToConstantHeading(new Vector2d(-60, 8), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(48.5, 35), Math.toRadians(90.00))
                .build();

        TrajectorySequence wait = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.2)
                .build();

        TrajectorySequence moveIn = drive.trajectorySequenceBuilder(traj2c.end())
                .strafeLeft(28)
                .back(10)
                .build();



        waitForStart();

        leftClawPos = leftClawClosed;
        rightClawPos = rightClawClosed;
        leftSwerverPos = leftSwerverVert;
        rightSwerverPos = rightSwerverVert;


        State currentState = State.IDLE;

        route = propPipeline.getLocation();

        if (route == 1) {
            currentState = State.TRAJECTORY_1;
            cycleStart = traj1c.end();
        } else if (route == 2) {
            currentState = State.TRAJECTORY_2;
            cycleStart = traj2c.end();
        } else if (route == 3) {
            currentState = State.TRAJECTORY_3;
            cycleStart = traj3c.end();
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
                        drive.followTrajectorySequenceAsync(traj1);
                        currentState = AutoBlueBack.State.TRAJECTORY_1C;
                    }
                    break;

                case TRAJECTORY_2:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(traj2);
                        currentState = AutoBlueBack.State.TRAJECTORY_2C;
                    }
                    break;

                case TRAJECTORY_3:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(traj3);
                        currentState = AutoBlueBack.State.TRAJECTORY_3C;
                    }
                    break;

                case TRAJECTORY_1C:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        extaking = true;
                        drive.followTrajectorySequenceAsync(traj1c);
                        currentState = AutoBlueBack.State.SLIDE_UP;
                    }
                    break;

                case TRAJECTORY_2C:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        extaking = true;
                        drive.followTrajectorySequenceAsync(traj2c);
                        currentState = AutoBlueBack.State.SLIDE_UP;
                    }
                    break;

                case TRAJECTORY_3C:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        extaking = true;
                        drive.followTrajectorySequenceAsync(traj3c);
                        currentState = AutoBlueBack.State.SLIDE_UP;
                    }
                    break;

                case SLIDE_UP:

                    extend = true;
                    currentState = AutoBlueBack.State.SWERVE_BACK;
                    break;

                case SWERVE_BACK:

                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(wait);
                        if (!leftSlide.isBusy() && !rightSlide.isBusy()) {
                            rightSwerverPos = rightSwerverDropper;
                            leftSwerverPos = leftSwerverDropper;
                            currentState = AutoBlueBack.State.BOX_OPEN;
                        }
                    }
                    break;

                case BOX_OPEN:
                    if ((leftSwerver.getPosition() == leftSwerverDropper) && (rightSwerver.getPosition() == rightSwerverDropper)) {
                        drive.followTrajectorySequence(wait);
                        leftClawPos = leftClawOpen;
                        rightClawPos = rightClawOpen;
                        currentState = AutoBlueBack.State.SWERVE_FORWARD;
                    }
                    break;

                case SWERVE_FORWARD:
                    drive.followTrajectorySequence(wait);
                    drive.followTrajectorySequence(wait);
                    drive.followTrajectorySequence(wait);
                    leftSwerverPos = leftSwerverPropel;
                    rightSwerverPos = rightSwerverPropel;
                    currentState = AutoBlueBack.State.SLIDE_DOWN;
                    break;


                case SLIDE_DOWN:
                    drive.followTrajectorySequence(wait);
                    drive.followTrajectorySequence(wait);
                    drive.followTrajectorySequence(wait);
                    drive.followTrajectorySequence(wait);
                    drive.followTrajectorySequence(wait);
                    extend = false;
                    currentState = AutoBlueBack.State.MOVE_IN;
                    break;


                case MOVE_IN:
                    drive.followTrajectorySequenceAsync(moveIn);
                    currentState = AutoBlueBack.State.IDLE;
                    break;


                case CYCLE_1:

                    if (!drive.isBusy()) {
                        extaking = false;
                        intaking = true;
                        drive.followTrajectorySequenceAsync(cycle);
                        currentState = AutoBlueBack.State.IDLE;
                    }

                    break;



                case IDLE:
                    intaking = false;
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }



            // We update drive continuously in the background, regardless of state
            drive.update();

            int leftPos = -leftSlide.getCurrentPosition();
            int rightPos = rightSlide.getCurrentPosition();

            if (((leftPos > 10 && leftPos < 1400) && (rightPos > 10 && rightPos < 1400)) && (extend)) {
                leftSwerverPos = leftSwerverPropel;
                rightSwerverPos = rightSwerverPropel;
            }

            if (((leftPos > 10 && leftPos < 600) && (rightPos > 10 && rightPos < 600)) && (!extend)) {
                leftSwerverPos = leftSwerverVert;
                rightSwerverPos = rightSwerverVert;
            }

            leftClaw.setPosition(leftClawPos);
            rightClaw.setPosition(rightClawPos);
            leftSwerver.setPosition(leftSwerverPos);
            rightSwerver.setPosition(rightSwerverPos);

            if (intaking) {
                intake.setTargetPosition(-5000);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(-0.8);
            }
            else if (extaking) {
                intake.setTargetPosition(200);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(0.25);
            }
            else {
                intake.setPower(0);
            }




            if (extend) {
                leftSlide.setTargetPosition(-1500);
                rightSlide.setTargetPosition(1500);
            }
            else {
                leftSlide.setTargetPosition(0);
                rightSlide.setTargetPosition(0);
            }
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(-1);
            rightSlide.setPower(1);


        }
    }
}