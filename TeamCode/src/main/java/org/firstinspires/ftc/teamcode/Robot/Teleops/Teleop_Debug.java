package org.firstinspires.ftc.teamcode.Robot.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.IndexerFacade;
import org.firstinspires.ftc.teamcode.Robot.LauncherFacade;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2026;
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

@TeleOp(group = Teleop_Debug.MATCH_TELEOP_GROUP)
public class Teleop_Debug extends OpMode {

    public static final String MATCH_TELEOP_GROUP = "AAAMatchTeleops";
    public TelemetryPacket p = new TelemetryPacket(true);
    // --- Mode States ---

    ThunderBot2026.Alliance_Color alliance = ThunderBot2026.Alliance_Color.BLUE;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    private TBDGamepad theGamepad1 = null;
    private TBDGamepad theGamepad2 = null;

    ThunderBot2026 robot = new ThunderBot2026();
    public static double INDEXER_SPEED = 0.8;
    private boolean preSpinUp = true;

    @Override
    public void init() {
        Pose2d pose = new Pose2d(0, 0, 0);

        robot.init(hardwareMap, telemetry, pose);

        theGamepad1 = new TBDGamepad(gamepad1);
        theGamepad2 = new TBDGamepad(gamepad2);

        // Tell the driver that initialization is complete.

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void start() {
        robot.runtime.reset();
        robot.setColor(alliance);
    }

    @Override
    public void loop() {
        robot.update();
        theGamepad1.update();
        theGamepad2.update();

        // --- Drive Controls ---
        if(theGamepad2.getButtonPressed(TBDGamepad.Button.X)){
//            robot.intake.DEBUG_intakeMotor();
        }
        if(theGamepad2.getButtonPressed(TBDGamepad.Button.Y) || theGamepad2.getButton(TBDGamepad.Button.DPAD_RIGHT))
        {
            robot.launcher.flywheelController.DEBUG_upperFlywheel();
        }

        if(theGamepad2.getButtonPressed(TBDGamepad.Button.A)){

        }

        if(theGamepad2.getButtonPressed(TBDGamepad.Button.B) || theGamepad2.getButton(TBDGamepad.Button.DPAD_LEFT))
        {
            robot.launcher.flywheelController.DEBUG_lowerFlywheel();
        }


        robot.drive.localizer.update();

        telemetry.addData("Flywheel Upper: ", robot.launcher.getUpperFlywheelRpm());
        telemetry.addData("Flywheel Lower: ", robot.launcher.getLowerFlywheelRpm());


        dashboard.sendTelemetryPacket(p);
        telemetry.update();
    }
}
