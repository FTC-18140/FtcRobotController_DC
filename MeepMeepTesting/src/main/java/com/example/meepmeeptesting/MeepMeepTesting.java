package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d launchPos = new Pose2d(new Vector2d(24, 16), Math.toRadians(0));
        Pose2d intakePos = new Pose2d(new Vector2d(13.5, 32), Math.toRadians(90));
        Pose2d intakePos2 = new Pose2d(new Vector2d(-10, 32), Math.toRadians(90));
        Pose2d intakePos3 = new Pose2d(new Vector2d(-34.5, 32), Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setStartPose(new Pose2d(58, 46, Math.toRadians(-45)))
                .setDimensions(15,17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(58, 46, Math.toRadians(-45)))
                        .setReversed(true)
                        .splineTo(launchPos.position, Math.toRadians(-90))

                        .waitSeconds(0.5)
                        .splineTo(intakePos.position, Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(intakePos.position.x, 49), Math.toRadians(90), new TranslationalVelConstraint(12))



                        .waitSeconds(0.5)
                        .setTangent(Math.toRadians(-90))
                        .setReversed(true)
                        .splineTo(launchPos.position, Math.toRadians(-90))



                        .waitSeconds(0.5)
                        .splineTo(intakePos2.position, Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(intakePos2.position.x, 49), Math.toRadians(90), new TranslationalVelConstraint(12))


                        .waitSeconds(0.5)

                        .setTangent(Math.toRadians(-90))
                        .splineTo(launchPos.position, Math.toRadians(-90))

                        .waitSeconds(0.5)
                        .splineTo(intakePos3.position, Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(intakePos3.position.x, 49), Math.toRadians(90), new TranslationalVelConstraint(12))


                        .waitSeconds(0.5)

                        .setTangent(Math.toRadians(-90))
                        .splineTo(launchPos.position, Math.toRadians(-90))


                        .waitSeconds(0.5)
                        .setReversed(true)
                        .splineTo(new Vector2d(38, 12), Math.toRadians(0))
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}
