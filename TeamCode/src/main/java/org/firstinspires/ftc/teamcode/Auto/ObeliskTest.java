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
public class ObeliskTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(AutoPositions.Positions.START_RED_FAR.position, Math.toRadians(0));

        ThunderBot2025 robot = new ThunderBot2025();
        blackboard.put("TURRET_ENDING_ANGLE_AUTO", (double) 0);
        blackboard.put("ENDING_ANGLE_INDEXER", (double) 0);

        robot.init(hardwareMap, telemetry, start);
        while (opModeInInit()) {
            // Code here runs repeatedly during init phase.  Need to be looking at ObeliskID
            robot.launcher.updateVision();
            robot.registerObeliskID();
            telemetry.addData("Status", "Waiting for start");
            telemetry.update();
        }
        waitForStart();

        robot.setColor(ThunderBot2025.Alliance_Color.RED);

        Actions.runBlocking(
                new ParallelAction(
                        robot.updateAction(),
                        robot.aimAction(),
                        robot.launcher.prepShotAction(),
                        new SequentialAction(
                                new SleepAction(1),
                                robot.startSequenceAction()
                        )
                )
        );

    }
}
