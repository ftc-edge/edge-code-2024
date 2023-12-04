package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    private DcMotor Vertical_slide = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        Vertical_slide = hardwareMap.get(DcMotor.class, "VertSlide");

        front_left = hardwareMap.get(DcMotor.class, "topleft");
        front_right = hardwareMap.get(DcMotor.class, "topright");
        back_left = hardwareMap.get(DcMotor.class, "bottomleft");
        back_right = hardwareMap.get(DcMotor.class, "bottomright");

        Vertical_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Vertical_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Vertical_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Vertical_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).

        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double twist = gamepad1.right_stick_x;

        double slideSpeed = -gamepad2.left_stick_y;

        if (slideSpeed > 1) {
            slideSpeed = 1;
        }
        if (slideSpeed < -1) {
            slideSpeed = -1;
        }

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

        int position = Vertical_slide.getCurrentPosition();
        telemetry.addData("Encoder Position", position);

        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);

        Vertical_slide.setPower(slideSpeed);

//        if (gamepad2.a) {
//            Vertical_slide.setTargetPosition(0.5);
//            Vertical_slide.setPower(1);
//        }
//        else {
//            Vertical_slide.setPower(slideSpeed);
//        }
    }
}