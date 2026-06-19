package org.firstinspires.ftc.teamcode.Robot.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.TurretController;
import org.firstinspires.ftc.teamcode.Utilities.LoopTime;
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

@TeleOp(name = "PID Aimer Test", group = "Testing")
@Config
public class Teleop_TurretTest extends OpMode {
    public static double STEP_CHANGE = 20;
//    Aimer turret = new Aimer();
    public MecanumDrive drive = null;
    public LoopTime loopTime = new LoopTime();

    TurretController turretController = new TurretController();

    private TBDGamepad gp2;
    private double testAngleChange = 0;

    @Override
    public void init()
    {
        // Essential: Sends data to Dashboard so you can see the PID graphs
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d pose = new Pose2d(0, 0, 0);

        turretController.init(hardwareMap, telemetry, pose);
        gp2 = new TBDGamepad(gamepad2);
        gp2.init(telemetry);

        drive = new MecanumDrive(hardwareMap, pose);
        loopTime.init();
    }

    @Override
    public void loop()
    {
        gp2.update();
        loopTime.update();

        /////////
        PoseVelocity2d robotPoseVel = drive.updatePoseEstimate();
        /////////

        if (gp2.getButtonPressed(TBDGamepad.Button.RIGHT_STICK_BUTTON))
        {
            turretController.toggleAimingMode();
        }
        if ( gp2.getButtonPressed(TBDGamepad.Button.DPAD_LEFT))
        {
            testAngleChange -= STEP_CHANGE;
            turretController.manuallyMoveTarget(-STEP_CHANGE);
        }
        else if ( gp2.getButtonPressed(TBDGamepad.Button.DPAD_RIGHT))
        {
            testAngleChange += STEP_CHANGE;
            turretController.manuallyMoveTarget(STEP_CHANGE);
        }
        else if (gp2.getButton(TBDGamepad.Button.RIGHT_BUMPER))
        {
            if (Math.hypot(gp2.getLeftX(), gp2.getLeftY()) > 0.2)
            {
                double newAngle = Math.toDegrees(Math.atan2(gp2.getLeftY(), gp2.getLeftX()));

                // ONLY call seekToAngle if the joystick has actually moved significantly
                // This is your "Dirty Flag" logic
                if (Math.abs(newAngle - testAngleChange) > 1.0)
                {
                    testAngleChange = newAngle;
//                    turretController.testMoveTarget(testAngleChange);
                }
            }
        }
        /////////
        turretController.update(drive.localizer.getPose(), robotPoseVel);
        /////////

        telemetry.update();
    }
}
