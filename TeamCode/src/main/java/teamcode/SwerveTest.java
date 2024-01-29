package teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="SwerveTest", group="Iterative Opmode")
public class SwerveTest extends LinearOpMode {
    private Servo leftSwerver = null;
    private Servo rightSwerver = null;
    public static double RswervePos = 1;
    public static double LswervePos = 1;



    @Override
    public void runOpMode() {
        leftSwerver = hardwareMap.get(Servo.class, "leftSwerver");
        rightSwerver = hardwareMap.get(Servo.class, "rightSwerver");




        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            rightSwerver.setPosition(RswervePos);
            leftSwerver.setPosition(LswervePos);
        }
    }
}

//below values are for swerver
//0.595 for against ground
//0.65 for dropping pixels