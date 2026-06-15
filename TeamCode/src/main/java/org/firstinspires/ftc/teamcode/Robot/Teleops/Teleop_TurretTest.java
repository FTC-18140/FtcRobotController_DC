package org.firstinspires.ftc.teamcode.Robot.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Aimer;
import org.firstinspires.ftc.teamcode.Robot.Drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.LauncherFacade;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2026;
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

@TeleOp(name = "PID Aimer Test", group = "Testing")
@Config
public class Teleop_TurretTest extends OpMode {
    public static double STEP_CHANGE = 20;
    Aimer turret = new Aimer();
    public MecanumDrive drive = null;

    private TBDGamepad gp2;
    private double testTargetAngle = 0;

    @Override
    public void init()
    {
        // Essential: Sends data to Dashboard so you can see the PID graphs
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turret.init(hardwareMap, telemetry);
        gp2 = new TBDGamepad(gamepad2);
        gp2.init(telemetry);
        testTargetAngle = turret.getCurrentAngle();

        Pose2d pose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, pose);

    }

    @Override
    public void loop() {
        gp2.update();
        PoseVelocity2d robotPoseVel = drive.updatePoseEstimate();

        if ( gp2.getButtonPressed(TBDGamepad.Button.DPAD_LEFT))
        {
            testTargetAngle -= STEP_CHANGE;
            turret.seekToAngle(testTargetAngle);
        }
        else if ( gp2.getButtonPressed(TBDGamepad.Button.DPAD_RIGHT))
        {
            testTargetAngle += STEP_CHANGE;
            turret.seekToAngle(testTargetAngle);
        }
        else if (gp2.getButton(TBDGamepad.Button.RIGHT_BUMPER))
        {
            if (Math.hypot(gp2.getLeftX(), gp2.getLeftY()) > 0.2)
            {
                double newAngle = Math.toDegrees(Math.atan2(gp2.getLeftY(), gp2.getLeftX()));

                // ONLY call seekToAngle if the joystick has actually moved significantly
                // This is your "Dirty Flag" logic
                if (Math.abs(newAngle - testTargetAngle) > 1.0)
                {
                    testTargetAngle = newAngle;
                    turret.seekToAngle(testTargetAngle);
                }
            }
        }

        turret.update(robotPoseVel); // Keep the robot internal states (IMU/Encoders) fresh

        telemetry.addData("Target Angle", testTargetAngle);
        telemetry.addData("Current Angle", turret.getCurrentAngle());

        telemetry.update();
    }
}
