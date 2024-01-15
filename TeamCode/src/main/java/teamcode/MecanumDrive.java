package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private Servo swerver = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    boolean intaking = true;
    boolean hooking = false;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.

        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        intake = hardwareMap.get(DcMotor.class, "intake");

        swerver = hardwareMap.get(Servo.class, "swerver");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        frontLeft = hardwareMap.get(DcMotor.class, "topleft");
        frontRight = hardwareMap.get(DcMotor.class, "topright");
        backLeft = hardwareMap.get(DcMotor.class, "bottomleft");
        backRight = hardwareMap.get(DcMotor.class, "bottomright");


//        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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


        swerver.setPosition(0.6385);
        leftClaw.setPosition(0.485);
        rightClaw.setPosition(0.43);


    }

    @Override
    public void loop() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).

        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
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
            leftClaw.setPosition(0.44);
            rightClaw.setPosition(0.465);
            swerver.setPosition(0.6385);
        }

        if (gamepad2.b) {
            swerver.setPosition(0.69);
        }

        if (gamepad2.left_trigger == 1) {
            leftClaw.setPosition(0.485);
            rightClaw.setPosition(0.42);
        }

        frontLeft.setPower(speeds[0]);
        frontRight.setPower(speeds[1]);
        backLeft.setPower(speeds[2]);
        backRight.setPower(speeds[3]);

        if (gamepad2.dpad_left) {
            hooking = true;
        }

        if (gamepad2.dpad_right) {
            hooking = false;
        }

        if (gamepad2.a) {
            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(-1);
            rightSlide.setPower(1);
        }

        else if (gamepad2.x) {
            leftSlide.setTargetPosition(-1000);
            rightSlide.setTargetPosition(1000);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(-1);
            rightSlide.setPower(1);
        }

        else if (gamepad2.y) {
            leftSlide.setTargetPosition(-2000);
            rightSlide.setTargetPosition(2000);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(-1);
            rightSlide.setPower(1);
        }

        else if (gamepad2.dpad_up) {
            leftSlide.setTargetPosition(-3000);
            rightSlide.setTargetPosition(3000);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(-1);
            rightSlide.setPower(1);
        }

        else if (hooking) {
            leftSlide.setPower(0.5);
            rightSlide.setPower(0.5);
        }



        if (gamepad1.right_bumper) {
            intaking = true;
        }

        if (gamepad1.left_bumper) {
            intaking = false;
        }

        if (intaking == true) {
            intake.setPower(-0.2);
        }

        else {
            intake.setPower(0.2);
        }


    }
}

//below values are for swerver
//0.595 for against ground
//0.65 for dropping pixels

