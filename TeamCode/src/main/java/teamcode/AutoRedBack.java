package teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
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

@Autonomous(name = "AutoRedBack")
public class AutoRedBack extends LinearOpMode {
    RedPropPipeline propPipeline;
    OpenCvCamera camera;
    double route;
    boolean extend = false;
    boolean miniExtend = false;
    boolean extaking = false;
    boolean intaking = false;
    boolean spit = false;
    boolean intakeCycleOut = false;
    boolean intakeCycleIn = false;
    Pose2d cycleStart = new Pose2d(48.50, -36.00, Math.toRadians(180));
    double leftClawPos;
    double rightClawPos;
    double leftSwerverPos;
    double rightSwerverPos;
    double rightClawOpen = 0.59;
    double rightClawClosed = 0.71;
    double leftClawOpen = 0.45;
    double leftClawClosed = 0.34;
    double rightSwerverDropper = 0.4;
    double rightSwerverVert = 0.09;
    double rightSwerverPropel = 0;
    double leftSwerverDropper = 0.4;
    double leftSwerverVert = 0.09;
    double leftSwerverPropel = 0;

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
        LOWER,
        UPPER,
        BOX_OPEN,      // open claw
        SWERVE_FORWARD,
        SLIDE_DOWN,
        MOVE_IN,
        CYCLE_1F,
        INTAKE_CYCLE,
        CYCLE_1B,
        SLIDE_UP2,       //bring up claw
        SWERVE_BACK2,
        LOWER2,
        UPPER2,
        BOX_OPEN2,      // open claw
        SWERVE_FORWARD2,
        SLIDE_DOWN2,
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

        propPipeline = new RedPropPipeline(telemetry);

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


        Pose2d startPose = new Pose2d(16, -56.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(6, -35.00,Math.toRadians(180)), Math.toRadians(180.00))
                .build();



        TrajectorySequence traj1c = drive.trajectorySequenceBuilder(traj1.end())
                .waitSeconds(0.2)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(52, -30.00),Math.toRadians(0))
                .build();




        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(22.00, -24.00,Math.toRadians(180)),Math.toRadians(90))
                .build();

        TrajectorySequence traj2c = drive.trajectorySequenceBuilder(traj2.end())
                .waitSeconds(0.2)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(52, -36.00), Math.toRadians(0.00))
                .build();



        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(23, -36.00), Math.toRadians(90.00))
                .build();

        TrajectorySequence traj3c = drive.trajectorySequenceBuilder(traj3.end())
                .waitSeconds(0.2)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(52, -40.00, Math.toRadians(180.00)),Math.toRadians(45))
                .build();



        TrajectorySequence cycleF = drive.trajectorySequenceBuilder(cycleStart)
                .setTangent(Math.toRadians(90))
                .setVelConstraint(new TranslationalVelocityConstraint(25))
                .splineToConstantHeading(new Vector2d(-24, 0), Math.toRadians(180.00))
                .splineToConstantHeading(new Vector2d(-61, 5), Math.toRadians(180.00))
                .splineToConstantHeading(new Vector2d(-61, -8), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(-50, -18), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(-63, -18), Math.toRadians(180.00))
                .waitSeconds(0.5)
                .setVelConstraint(new TranslationalVelocityConstraint(25))
                .splineToConstantHeading(new Vector2d(-59, -18), Math.toRadians(180.00))
                .splineToConstantHeading(new Vector2d(-63, -18), Math.toRadians(180.00))
                .waitSeconds(0.5)
                .setVelConstraint(new TranslationalVelocityConstraint(25))
                .splineToConstantHeading(new Vector2d(-59, -18), Math.toRadians(180.00))
                .splineToConstantHeading(new Vector2d(-63, -18), Math.toRadians(180.00))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence cycleB = drive.trajectorySequenceBuilder(cycleF.end())
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-20, 0), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(30, -10), Math.toRadians(0))
                .setVelConstraint(new TranslationalVelocityConstraint(10))
                .splineToConstantHeading(new Vector2d(52, -36), Math.toRadians(315.00))
                .build();

        TrajectorySequence wait = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.2)
                .build();

        TrajectorySequence waitLong = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(2)
                .build();

        TrajectorySequence moveIn = drive.trajectorySequenceBuilder(cycleStart)
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(45.00, -48.00), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(56.00, -60.00), Math.toRadians(0.00))
                .build();




        waitForStart();

        leftClawPos = leftClawClosed;
        rightClawPos = rightClawClosed;
        leftSwerverPos = leftSwerverVert;
        rightSwerverPos = rightSwerverVert;


        AutoRedBack.State currentState = AutoRedBack.State.IDLE;

        route = propPipeline.getLocation();

        if (route == 1) {
            currentState = AutoRedBack.State.TRAJECTORY_1;
            cycleStart = traj1c.end();
        } else if (route == 2) {
            currentState = AutoRedBack.State.TRAJECTORY_2;
            cycleStart = traj2c.end();
        } else if (route == 3) {
            currentState = AutoRedBack.State.TRAJECTORY_3;
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
                        currentState = State.TRAJECTORY_1C;
                    }
                    break;

                case TRAJECTORY_2:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(traj2);
                        currentState = State.TRAJECTORY_2C;
                    }
                    break;

                case TRAJECTORY_3:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(traj3);
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
                        extaking = true;
                        drive.followTrajectorySequenceAsync(traj1c);
                        currentState = AutoRedBack.State.SLIDE_UP;
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
                        currentState = AutoRedBack.State.SLIDE_UP;
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
                        currentState = AutoRedBack.State.SLIDE_UP;
                    }
                    break;

                case SLIDE_UP:

                    extend = true;
                    currentState = AutoRedBack.State.SWERVE_BACK;
                    break;

                case SWERVE_BACK:

                    if (!drive.isBusy()) {
                        if (!leftSlide.isBusy() || !rightSlide.isBusy()) {
                            rightSwerverPos = rightSwerverDropper;
                            leftSwerverPos = leftSwerverDropper;
                            currentState = AutoRedBack.State.LOWER;
                        }
                    }
                    break;

                case LOWER:

                    drive.followTrajectorySequence(wait);
                    drive.followTrajectorySequence(wait);
                    miniExtend = true;
                    currentState = AutoRedBack.State.BOX_OPEN;


                    break;


                case BOX_OPEN:
                    if (!leftSlide.isBusy() || !rightSlide.isBusy()) {
                        drive.followTrajectorySequence(wait);
                        drive.followTrajectorySequence(wait);
                        rightClawPos = rightClawOpen;
                        leftClawPos = leftClawOpen;
                        currentState = AutoRedBack.State.UPPER;
                    }

                    break;

                case UPPER:

                    drive.followTrajectorySequence(wait);
                    drive.followTrajectorySequence(wait);
                    miniExtend = false;
                    currentState = AutoRedBack.State.SWERVE_FORWARD;


                    break;

                case SWERVE_FORWARD:
                    if (!leftSlide.isBusy() || !rightSlide.isBusy()) {
                        drive.followTrajectorySequence(wait);
                        drive.followTrajectorySequence(wait);
                        leftSwerverPos = leftSwerverPropel;
                        rightSwerverPos = rightSwerverPropel;
                        currentState = AutoRedBack.State.SLIDE_DOWN;
                    }

                    break;


                case SLIDE_DOWN:
                    drive.followTrajectorySequence(wait);
                    drive.followTrajectorySequence(wait);
                    drive.followTrajectorySequence(wait);
                    extend = false;
                    currentState = AutoRedBack.State.MOVE_IN;
                    break;


                case MOVE_IN:
                    drive.followTrajectorySequenceAsync(moveIn);
                    currentState = AutoRedBack.State.IDLE;
                    break;


                case CYCLE_1F:

                    if (!drive.isBusy()) {
                        extaking = false;
                        intaking = true;
                        drive.followTrajectorySequenceAsync(cycleF);
                        intakeCycleOut = true;
                        currentState = State.INTAKE_CYCLE;
                    }

                    break;

                case INTAKE_CYCLE:

                    if (!drive.isBusy()) {
                        intaking = false;
                        drive.followTrajectorySequenceAsync(waitLong);
                        currentState = State.CYCLE_1B;
                    }

                case CYCLE_1B:

                    if (!drive.isBusy()) {
                        leftClawPos = leftClawClosed;
                        rightClawPos = rightClawClosed;
                        intaking = false;
                        spit = true;
                        drive.followTrajectorySequenceAsync(cycleB);
                        currentState = AutoRedBack.State.SLIDE_UP2;
                    }

                    break;

                case SLIDE_UP2:

                    if (!drive.isBusy()) {
                        extend = true;
                        currentState = AutoRedBack.State.SWERVE_BACK2;
                    }
                    break;

                case SWERVE_BACK2:

                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(wait);
                        drive.followTrajectorySequence(wait);
                        drive.followTrajectorySequence(wait);
                        if (!leftSlide.isBusy() || !rightSlide.isBusy()) {
                            rightSwerverPos = rightSwerverDropper;
                            leftSwerverPos = leftSwerverDropper;
                            currentState = AutoRedBack.State.LOWER2;
                        }
                    }
                    break;

                case LOWER2:

                    drive.followTrajectorySequence(wait);
                    drive.followTrajectorySequence(wait);
                    miniExtend = true;
                    currentState = AutoRedBack.State.BOX_OPEN2;


                    break;


                case BOX_OPEN2:
                    if (!leftSlide.isBusy() || !rightSlide.isBusy()) {
                        drive.followTrajectorySequence(wait);
                        drive.followTrajectorySequence(wait);
                        rightClawPos = rightClawOpen;
                        leftClawPos = leftClawOpen;
                        currentState = AutoRedBack.State.UPPER2;
                    }

                    break;

                case UPPER2:

                    drive.followTrajectorySequence(wait);
                    drive.followTrajectorySequence(wait);
                    miniExtend = false;
                    currentState = AutoRedBack.State.SWERVE_FORWARD2;


                    break;

                case SWERVE_FORWARD2:
                    if (!leftSlide.isBusy() || !rightSlide.isBusy()) {
                        drive.followTrajectorySequence(wait);
                        drive.followTrajectorySequence(wait);
                        leftSwerverPos = leftSwerverPropel;
                        rightSwerverPos = rightSwerverPropel;
                        currentState = AutoRedBack.State.SLIDE_DOWN2;
                    }

                    break;


                case SLIDE_DOWN2:
                    drive.followTrajectorySequence(wait);
                    drive.followTrajectorySequence(wait);
                    drive.followTrajectorySequence(wait);
                    extend = false;
                    currentState = AutoRedBack.State.IDLE;
                    break;



                case IDLE:
                    if (!drive.isBusy()) {
                        intaking = false;
                        extaking = false;
                        spit = false;
                        intakeCycleOut = false;
                        intakeCycleIn = false;
                    }
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }



            // We update drive continuously in the background, regardless of state
            drive.update();

            int leftPos = -leftSlide.getCurrentPosition();
            int rightPos = rightSlide.getCurrentPosition();

            if (((leftPos > 10 && leftPos < 600) && (rightPos > 10 && rightPos < 600)) && (extend) && (!miniExtend)) {
                leftSwerverPos = leftSwerverPropel;
                rightSwerverPos = rightSwerverPropel;
            }

            if (((leftPos > 10 && leftPos < 600) && (rightPos > 10 && rightPos < 600)) && (!extend) && (!miniExtend)) {
                leftSwerverPos = leftSwerverVert;
                rightSwerverPos = rightSwerverVert;
            }

            leftClaw.setPosition(leftClawPos);
            rightClaw.setPosition(rightClawPos);
            leftSwerver.setPosition(leftSwerverPos);
            rightSwerver.setPosition(rightSwerverPos);

            if (intaking) {
                intake.setTargetPosition(-25000);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(-1);
            }
            else if (extaking) {
                intake.setTargetPosition(40);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(0.2);
            }

            else if (spit) {
                intake.setTargetPosition(-5000);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(0.6);
            }

            else if (intakeCycleOut) {
                intake.setTargetPosition(-20000);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(1);
                if (!intake.isBusy()) {
                    intakeCycleOut = false;
                    intakeCycleIn = true;
                }
            }

            else if (intakeCycleIn) {
                intake.setTargetPosition(-21000);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(-1);
                if (!intake.isBusy()) {
                    intakeCycleIn = false;
                    intakeCycleOut = true;
                }
            }
            else {
                intake.setPower(0);
            }


            if (miniExtend && extend) {
                leftSlide.setTargetPosition(-1000);
                rightSlide.setTargetPosition(1000);
            }

            else if (extend) {
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