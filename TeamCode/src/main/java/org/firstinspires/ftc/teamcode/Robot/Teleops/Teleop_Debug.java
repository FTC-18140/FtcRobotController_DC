package org.firstinspires.ftc.teamcode.Robot.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.IndexerFacade;
import org.firstinspires.ftc.teamcode.Robot.LauncherFacade;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

@TeleOp(group = Teleop_Debug.MATCH_TELEOP_GROUP)
@Config
public class Teleop_Debug extends OpMode {

    public static final String MATCH_TELEOP_GROUP = "AAAMatchTeleops";
    public TelemetryPacket p = new TelemetryPacket(true);
    // --- Mode States ---
    private boolean isAutoLoading = false;
    private int slotToWatch = -1;

    ThunderBot2025.Alliance_Color alliance = ThunderBot2025.Alliance_Color.BLUE;
    TBDGamepad.Colors gamepad1Color = null;
    TBDGamepad.Colors gamepad2Color = null;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    private TBDGamepad theGamepad1 = null;
    private TBDGamepad theGamepad2 = null;

    ThunderBot2025 robot = new ThunderBot2025();
    public static double INDEXER_SPEED = 0.8;
    private boolean preSpinUp = true;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, null);

        theGamepad1 = new TBDGamepad(gamepad1);
        theGamepad2 = new TBDGamepad(gamepad2);
        theGamepad1.setLedColor(TBDGamepad.Colors.ORANGE);
        theGamepad2.setLedColor(TBDGamepad.Colors.BLUE);
        // Tell the driver that initialization is complete.

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        theGamepad1.setLedColor(TBDGamepad.Colors.YELLOW);
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
            robot.intake.DEBUG_intakeMotor();
        }
        if(theGamepad2.getButtonPressed(TBDGamepad.Button.Y)){
            robot.launcher.flywheel.DEBUG_upperFlywheel();
        }
        if(theGamepad2.getButtonPressed(TBDGamepad.Button.A)){

        }
        if(theGamepad2.getButtonPressed(TBDGamepad.Button.B)){
            robot.launcher.flywheel.DEBUG_lowerFlywheel();
        }


        robot.drive.localizer.update();

        telemetry.addData("Flywheel Upper: ", robot.launcher.getUpperFlywheelRpm());


        dashboard.sendTelemetryPacket(p);
    }
}
