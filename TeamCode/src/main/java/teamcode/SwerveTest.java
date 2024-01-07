package teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="SwerveTest", group="Iterative Opmode")
public class SwerveTest extends LinearOpMode {
    private Servo swerver = null;
    public static double swervePos = 1;



    @Override
    public void runOpMode() {
        swerver = hardwareMap.get(Servo.class, "swerver");


        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            swerver.setPosition(swervePos);
        }
    }
}

//below values are for swerver
//0.595 for against ground
//0.65 for dropping pixels