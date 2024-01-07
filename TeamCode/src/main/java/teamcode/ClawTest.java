package teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="ClawTest", group="Iterative Opmode")
public class ClawTest extends LinearOpMode {
    private Servo leftClaw = null;
    private Servo rightClaw = null;

    public static double leftPos = 1;
    public static double rightPos = 1;


    @Override
    public void runOpMode() {
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            leftClaw.setPosition(leftPos);
            rightClaw.setPosition(rightPos);
        }



    }


}


//0.465 right pos closed
//0.45 right pos open

//0.465 left pos closed
//0.45 left pos open