package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;

@Autonomous
public class AutoBlueDepot extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(AutoPositions.Positions.START_BLUE_DEPOT.position, Math.toRadians(135));

        ThunderBot2025 robot = new ThunderBot2025();

        robot.init(hardwareMap, telemetry, start);
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        robot.drive.actionBuilder(start)
                                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(0))
                                .build()
                )
        );
    }
}
