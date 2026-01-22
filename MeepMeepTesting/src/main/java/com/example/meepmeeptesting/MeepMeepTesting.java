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

        Pose2d launchPos = new Pose2d(-52, 16, Math.toRadians(0));
        Pose2d intakePos = new Pose2d(new Vector2d(-34.5, 32), Math.toRadians(90));
        Pose2d intakePos2 = new Pose2d(new Vector2d(-10, 32), Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setStartPose(new Pose2d(-63, 16, Math.toRadians(0)))
                .setDimensions(17,17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-63, 16, Math.toRadians(0)))
                        .splineToSplineHeading(launchPos, 0)

                        .waitSeconds(0.5)
                        .splineToSplineHeading(intakePos, Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-34.5, 49), Math.toRadians(90), new TranslationalVelConstraint(7))



                        .waitSeconds(0.5)
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(launchPos, Math.toRadians(180))



                        .waitSeconds(0.5)
                        .splineToSplineHeading(intakePos2, Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(intakePos2.position.x, 49), Math.toRadians(90), new TranslationalVelConstraint(7))


                        .waitSeconds(0.5)

                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(launchPos, Math.toRadians(180))


                        .waitSeconds(0.5)
                        .splineToSplineHeading(new Pose2d(-12, 12, 0), Math.toRadians(0))
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(7, -42), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(7, -31), Math.toRadians(90))
//
//                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(10, -48), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(48, -44), Math.toRadians(0))
//
//                .strafeToSplineHeading(new Vector2d(56, -52), Math.toRadians(135))
//
//                .strafeToSplineHeading(new Vector2d(60, -43), Math.toRadians(90))
//
//                .strafeToSplineHeading(new Vector2d(56, -52), Math.toRadians(135))
//
//                .strafeToSplineHeading(new Vector2d(55.5, -33), Math.toRadians(45))
//
//                .strafeToSplineHeading(new Vector2d(56, -52), Math.toRadians(135))
//
//                .setTangent(Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(52, -48, Math.toRadians(-90)), Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(48, -50.5), Math.toRadians(-90))
//
//                .setTangent(Math.toRadians(170))
//
//                .splineToSplineHeading(new Pose2d(7, -42, Math.toRadians(90)), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(7, -31), Math.toRadians(90))
//
//                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(14, -48), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(46, -50), Math.toRadians(0))
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}
