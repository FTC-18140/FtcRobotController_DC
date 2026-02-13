package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Graphics;
import java.awt.Image;
import java.awt.font.ImageGraphicAttribute;
import java.awt.image.ImageObserver;
import java.awt.image.ImageProducer;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.FileSystem;
import java.nio.file.Files;

import javax.imageio.ImageIO;
import javax.imageio.ImageReadParam;
import javax.imageio.ImageReader;
import javax.imageio.spi.ImageInputStreamSpi;
import javax.imageio.stream.ImageInputStream;

public class MeepMeepTesting {
    public static void main(String[] args) {


        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d launchPos1 = new Pose2d(new Vector2d(-56, -16), Math.toRadians(0));
        Pose2d launchPos2 = new Pose2d(new Vector2d(18, -16), Math.toRadians(0));
        Pose2d intakePos = new Pose2d(new Vector2d(-34.5, -26), Math.toRadians(-90));
        Pose2d intakePos2 = new Pose2d(new Vector2d(-10, -24), Math.toRadians(-90));
        Pose2d intakePos3 = new Pose2d(new Vector2d(13.5, -26), Math.toRadians(-90));
        Pose2d gatePos = new Pose2d(new Vector2d(2, 52.5), Math.toRadians(-90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setStartPose(new Pose2d(new Vector2d(-63,-16), Math.toRadians(0)))
                .setDimensions(15,17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(-63,-16), Math.toRadians(0)))

                        .splineTo(launchPos1.position, 0)

                        .waitSeconds(0.5)
                        .splineTo(intakePos.position, Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(intakePos.position.x, -53), Math.toRadians(-90), new TranslationalVelConstraint(20))

                        .setReversed(true)
                        .splineTo(launchPos1.position, Math.toRadians(180))


                        .waitSeconds(0.5)
                        .splineTo(intakePos2.position, Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(intakePos2.position.x, -53), Math.toRadians(-90), new TranslationalVelConstraint(20))


                        .waitSeconds(0.5)

                        .setReversed(true)
                        .splineToSplineHeading(launchPos2, Math.toRadians(0))


                        .waitSeconds(0.5)
                        .splineTo(intakePos3.position, Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(intakePos3.position.x, -53), Math.toRadians(-90), new TranslationalVelConstraint(20))

                        .waitSeconds(0.5)
                        .setReversed(true)
                .splineToSplineHeading(launchPos2, Math.toRadians(0))

                .waitSeconds(0.5)
                .setReversed(true)
                .splineTo(new Vector2d(-12, -12), Math.toRadians(180))


                .build()
        );

        try {
            meepMeep.setBackground(ImageIO.read(new File("decodefield.png")))
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        } catch (IOException e) {
            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }

    }
}
