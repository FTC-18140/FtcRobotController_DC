package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(15, -60, Math.toRadians(90)))
                .strafeTo(new Vector2d(7, -56))
                .strafeTo(new Vector2d(7, -31))

                .strafeTo(new Vector2d(10, -54))

                .strafeToSplineHeading(new Vector2d(48,-42), Math.toRadians(90))

                .turnTo(Math.toRadians(-60))

                .turnTo(Math.toRadians(60))

                .turnTo(Math.toRadians(-60))

                .strafeToSplineHeading(new Vector2d(52,-42), Math.toRadians(45))

                .turn(Math.toRadians(225))

                .strafeTo(new Vector2d(52, -42+12))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
