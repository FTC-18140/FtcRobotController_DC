package org.firstinspires.ftc.teamcode.Robot.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Aimer;
import org.firstinspires.ftc.teamcode.Robot.LauncherFacade;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2026;
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

@TeleOp(name = "PID Aimer Test", group = "Testing")
@Config
public class Teleop_TurretTest extends OpMode {
    Aimer theTurret = new Aimer();
    private TBDGamepad gp2;
    private double testTargetAngle = 0;

    @Override
    public void init() {
        // Essential: Sends data to Dashboard so you can see the PID graphs
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        theTurret.init(hardwareMap, telemetry);
        gp2 = new TBDGamepad(gamepad2);
        gp2.init(telemetry);
    }

    @Override
    public void loop() {
        theTurret.update(); // Keep the robot internal states (IMU/Encoders) fresh
        gp2.update();

        // 1. MANUAL TARGET CONTROL (Left Stick)
        // We move the "Target" manually so we can see how the PID reacts to changes
        if (Math.hypot(gp2.getLeftX(), gp2.getLeftY()) > 0.2) {
            testTargetAngle = Math.toDegrees(Math.atan2(gp2.getLeftY(), gp2.getLeftX()));
        }

        // 3. EXECUTION
        if (robot.launcher.getAimingMode() == LauncherFacade.AimingMode.MAIN) {
            // This calls your Aimer.java logic inside LauncherFacade
            robot.launcher.aimToAngleInFieldSpace(testTargetAngle);
        } else {
            // Manual safety: hold position if not testing PID
            robot.launcher.holdTurretPosition();
        }

        // 4. PID MONITORING TELEMETRY
        telemetry.addData("MODE", robot.launcher.getAimingMode());
        telemetry.addData("Target Angle", testTargetAngle);
        telemetry.addData("Actual Angle", robot.launcher.getTurretHeading());

        // IMPORTANT: If your Aimer class has a 'getPIDError()' or similar, add it here:
        // telemetry.addData("PID Error", robot.launcher.getAimerDebug());

        telemetry.update();
    }
}
