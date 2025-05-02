package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryActionFactory;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.IntakeClaw;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2024;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot_Claw_2024;

@Config
@Autonomous
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ThunderBot_Claw_2024 robot = new ThunderBot_Claw_2024();
        robot.init(hardwareMap,telemetry, 0);
        Pose2d startPos = new Pose2d(-15,-60,Math.toRadians(90));
        robot.drive.pose = startPos;
        TrajectoryActionBuilder tra1 = robot.drive.actionBuilder(startPos)
                .strafeTo(new Vector2d(startPos.position.x + 10, startPos.position.y), new TranslationalVelConstraint(4.0));
        Action finalTra = tra1.endTrajectory().fresh()
                .strafeTo(startPos.position)
                .build();

        telemetry.update();
        waitForStart();
        robot.intake.start();
        Actions.runBlocking(
                new ParallelAction(
                        robot.updatePixyAction(),
                    new SequentialAction(
                            robot.intake.presetAction(IntakeClaw.Positions.READY_TO_INTAKE),
                            robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                            robot.intake.armUpAction(29),
                            robot.intake.elbowAction(15),
                            new SleepAction(1),
                            robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                            new RaceAction(
                                    robot.alignToColorAction(50)
                            ),
                            new InstantAction(() -> robot.led.setToColor("yellow")),
                            robot.intake.elbowAction(IntakeClaw.Positions.READY_TO_INTAKE.elbowPos),
                            new SleepAction(0.5),
                            robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                            new SleepAction(0.5),
                            robot.intake.presetAction(IntakeClaw.Positions.READY_TO_INTAKE)

                    ),
                    robot.intake.updateAction()
                )
        );
    }
}
