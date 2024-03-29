package teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

import teamcode.util.Encoder;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.9448819; // in
    public static double X_MULTIPLIER = 1; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10.5; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -10; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    private List<Integer> lastEncPositions, lastEncVels;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "topright"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "bottomright"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "bottomleft"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos) * X_MULTIPLIER,
                encoderTicksToInches(rightPos) * X_MULTIPLIER,
                encoderTicksToInches(frontPos) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel) * X_MULTIPLIER,
                encoderTicksToInches(rightVel) * X_MULTIPLIER,
                encoderTicksToInches(frontVel) * Y_MULTIPLIER
        );
    }
}


//#1 is heading (deg): 356.48503933199896
//headingError (deg): 0.0
//x: -110.51487018007984
//xError: 0.0
//y: 2.088623957475418
//yError: 0.0

//#2 is heading (deg): 359.20629920399995
//headingError (deg): 0.0
//x: -109.93298886422986
//xError: 0.0
//y: 0.6128878570800523
//yError: 0.0

//#3 is heading (deg): 0.5669291399993757
//headingError (deg): 0.0
//x: -110.05648880643415
//xError: 0.0
//y: 0.939189014383832
//yError: 0.0



//lateral #1 heading (deg): 2.9156355771446116
//headingError (deg): 0.0
//x: 2.1257345423445186
//xError: 0.0
//y: -109.33664066348047
//yError: 0.0

//lateral #2 heading (deg): 0.5345331891427556
//headingError (deg): 0.0
//x: -3.0095631527631292
//xError: 0.0
//y: -110.14493262492086
//yError: 0.0

//lateral #3 heading (deg): 356.533633258283
//headingError (deg): 0.0
//x: -5.943870654092646
//xError: 0.0
//y: -111.27082914620665
//yError: 0.0


//lateral distance 10.31232