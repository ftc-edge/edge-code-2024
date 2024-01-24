package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(207.3827420689655), Math.toRadians(207.3827420689655), 14.73)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(48.5, 35, Math.toRadians(180)))
                                .setTangent(Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(-58, 0), Math.toRadians(180.00))
                                .splineToConstantHeading(new Vector2d(-60, 8), Math.toRadians(90.00))
                                .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(48.5, 35), Math.toRadians(90.00))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}