package teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="SlideTest", group="Iterative Opmode")
public class SlideTest extends LinearOpMode {
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;
    private DcMotor intake = null;
    public static int leftPos = 1;
    public static int rightPos = 1;

    public static double intakePower = 0.1;

    public static int intakePos = 1;

    public static double leftPower = 0;
    public static double rightPower = 0;
    int leftPosition;
    int rightPosition;


    @Override
    public void runOpMode() {
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();




            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setTargetPosition(intakePos);
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setPower(intakePower);
            while (intake.isBusy()) {}

//            leftSlide.setTargetPosition(leftPos);
//            leftSlide.setPower(leftPower);
//            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightSlide.setTargetPosition(rightPos);
//            rightSlide.setPower(rightPower);
//            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//            while (leftSlide.isBusy() || rightSlide.isBusy()) {}
//
//            leftPosition = leftSlide.getCurrentPosition();
//            rightPosition = rightSlide.getCurrentPosition();
//
//            telemetry.addData("left slide encoder", leftPosition);
//            telemetry.addData("right slide encoder", rightPosition);




    }


}