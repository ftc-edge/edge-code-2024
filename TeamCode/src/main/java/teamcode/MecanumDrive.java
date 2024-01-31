package teamcode;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @author Brandon Gong
 */
@TeleOp(name="Mecanum Drive", group="Iterative Opmode")
public class MecanumDrive extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;
    private DcMotor intake = null;
    private Servo leftSwerver = null;
    private Servo rightSwerver = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo planeLauncher = null;
    boolean intaking = false;
    double rightClawOpen = 0.5;
    double rightClawClosed = 0.7;
    double leftClawOpen = 0.47;
    double leftClawClosed = 0.27;
    double rightSwerverUp = 0.87;
    double rightSwerverDropper = 0.94;
    double rightSwerverVert = 0.887;
    double rightSwerverPropel = 0.85;
    double leftSwerverUp = 0.89;
    double leftSwerverDropper = 0.96;
    double leftSwerverVert = 0.907;
    double leftSwerverPropel = 0.87;
    double leftSwerverPos;
    double rightSwerverPos;
    int leftSlidePos = 0;
    int rightSlidePos = 0;
    double drive = 0;
    double strafe = 0;



    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.

        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftSwerver = hardwareMap.get(Servo.class, "leftSwerver");
        rightSwerver = hardwareMap.get(Servo.class, "rightSwerver");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");

        frontLeft = hardwareMap.get(DcMotor.class, "topleft");
        frontRight = hardwareMap.get(DcMotor.class, "topright");
        backLeft = hardwareMap.get(DcMotor.class, "bottomleft");
        backRight = hardwareMap.get(DcMotor.class, "bottomright");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);



        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftSwerver.setPosition(leftSwerverUp);
        rightSwerver.setPosition(rightSwerverUp);
        leftClaw.setPosition(leftClawOpen);
        rightClaw.setPosition(rightClawOpen);
        leftSwerverPos = leftSwerverUp;
        rightSwerverPos = rightSwerverUp;


    }

    @Override
    public void loop() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).

        if ((Math.abs(gamepad1.left_stick_y) < 0.4) && (Math.abs(gamepad1.left_stick_x) < 0.4)) {
            drive = gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
        }
        else {
            drive = 0;
            strafe = 0;
        }

        double twist = gamepad1.right_stick_x;


        int leftPos = -leftSlide.getCurrentPosition();
        int rightPos = rightSlide.getCurrentPosition();

        telemetry.addData("left slide encoder", leftPos);
        telemetry.addData("right slide encoder", rightPos);








        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (drive + strafe + twist),
                (-(drive - strafe - twist)),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }



        // apply the calculated values to the motors.



        if (gamepad2.right_trigger == 1) {
            leftClaw.setPosition(leftClawClosed);
            rightClaw.setPosition(rightClawClosed);
        }

        if (gamepad2.b) {
            leftSwerverPos = leftSwerverDropper;
            rightSwerverPos = rightSwerverDropper;
        }

        if (gamepad2.right_bumper) {
            leftSwerverPos = leftSwerverPropel;
            rightSwerverPos = rightSwerverPropel;
        }

        if (gamepad2.left_trigger == 1) {
            leftClaw.setPosition(leftClawOpen);
            rightClaw.setPosition(leftClawOpen);
        }

        if (gamepad2.a) {
            leftSlidePos = 0;
            rightSlidePos = 0;
        }

        else if (gamepad2.x) {
            leftSlidePos = -1500;
            rightSlidePos = 1500;
        }

        else if (gamepad2.y) {
            leftSlidePos = -2000;
            rightSlidePos = 2000;
        }

        else if (gamepad2.dpad_up) {
            leftSlidePos = -3000;
            rightSlidePos = 3000;
        }


        if (gamepad1.right_bumper) {
            intaking = true;
        }

        if (gamepad1.left_bumper) {
            intaking = false;
        }

        if (leftSlide.isBusy() || rightSlide.isBusy()) {
            intaking = false;
        }

        if (gamepad1.dpad_down) {
            planeLauncher.setPosition(0.7);
        }

//        if ((leftSwerver.getPosition() == leftSwerverDropper) && (rightSwerver.getPosition() == rightSwerverDropper)) {
//            leftClaw.setPosition(leftClawOpen);
//            rightClaw.setPosition(rightClawOpen);
//            if ((leftClaw.getPosition() == leftClawOpen) && (rightClaw.getPosition() == rightClawOpen)) {
//                leftSwerverPos = leftSwerverPropel;
//                rightSwerverPos = rightSwerverPropel;
//            }
//        }
//
//        if ((leftSwerver.getPosition() == leftSwerverPropel) && (rightSwerver.getPosition() == rightSwerverPropel)) {
//            leftSlidePos = 0;
//            rightSlidePos = 0;
//        }
//





        if (leftPos < 10 && rightPos < 10) {
            leftSwerverPos = leftSwerverUp;
            rightSwerverPos = rightSwerverUp;
        }

        if (((leftPos > 10 && leftPos < 1400) && (rightPos > 10 && rightPos < 1400)) && ((leftSlidePos == 0) && (rightSlidePos == 0))) {
            leftSwerverPos = leftSwerverVert;
            rightSwerverPos = rightSwerverVert;
        }

        if (((leftPos > 10 && leftPos < 1400) && (rightPos > 10 && rightPos < 1400)) && (!(leftSwerver.getPosition() == leftSwerverPropel))) {
            leftSlidePos = 2000;
            rightSlidePos = 2000;
        }

        if (intaking) {
            intake.setPower(-0.8);
        }

        else {
            intake.setPower(0);
        }

        leftSwerver.setPosition(leftSwerverPos);
        rightSwerver.setPosition(rightSwerverPos);

        frontLeft.setPower(speeds[0]);
        frontRight.setPower(speeds[1]);
        backLeft.setPower(speeds[2] * 0.85);
        backRight.setPower(speeds[3] * 0.85);

        leftSlide.setTargetPosition(leftSlidePos);
        rightSlide.setTargetPosition(rightSlidePos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(-1);
        rightSlide.setPower(1);

    }
}

