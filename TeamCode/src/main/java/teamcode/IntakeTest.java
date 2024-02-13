package teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="IntakeTest", group="Iterative Opmode")
public class IntakeTest extends LinearOpMode {
    private DcMotor intake;
    public static double intakePower = 1;
    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "intake");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
           intake.setPower(intakePower);
        }
    }
}