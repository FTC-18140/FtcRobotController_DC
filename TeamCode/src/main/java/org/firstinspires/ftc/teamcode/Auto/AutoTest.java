package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;

@Autonomous
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(AutoPositions.Positions.START_RED_FAR.position, Math.toRadians(0));

        ThunderBot2025 robot = new ThunderBot2025();

        robot.init(hardwareMap, telemetry, start);
        waitForStart();

        robot.setColor(ThunderBot2025.Alliance_Color.RED);

        Actions.runBlocking(
                new ParallelAction(
                        robot.drive.actionBuilder(start)
                                .strafeTo(new Vector2d(-12, -12))
                                .build(),
                        new SleepAction(5),
                        robot.updateAction(),
                        robot.aimAction()
                )
        );

        robot.setStartPosForTeleop(robot.drive.localizer.getPose());

    }
}
