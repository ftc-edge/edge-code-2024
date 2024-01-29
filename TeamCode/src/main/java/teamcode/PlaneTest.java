package teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="PlaneTest", group="Iterative Opmode")
public class PlaneTest extends LinearOpMode {
    private Servo planeLauncher = null;
    public static double LauncherPos = 1;


    @Override
    public void runOpMode() {
        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");




        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            planeLauncher.setPosition(LauncherPos);
        }
    }
}

//below values are for swerver
//0.595 for against ground
//0.65 for dropping pixels