package org.firstinspires.ftc.teamcode.Robot.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.IndexerFacade;
import org.firstinspires.ftc.teamcode.Robot.LauncherFacade;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;
import org.firstinspires.ftc.teamcode.Robot.TransferFacade;
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

@TeleOp(group = Teleop_BLUE.MATCH_TELEOP_GROUP)
@Disabled
public class Teleop_BLUE extends OpMode {

    public static final String MATCH_TELEOP_GROUP = "AAAMatchTeleops";
    public TelemetryPacket p = new TelemetryPacket(true);
    // --- Mode States ---
    private boolean isAutoLoading = false;
    private int slotToWatch = -1;

    ThunderBot2025.Alliance_Color alliance = ThunderBot2025.Alliance_Color.BLUE;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    private TBDGamepad theGamepad1 = null;
    private TBDGamepad theGamepad2 = null;

    ThunderBot2025 robot = new ThunderBot2025();
    public static double INDEXER_SPEED = 0.8;
    private boolean preSpinUp = true;
    double manualAngle;

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
//        switch (robot.lastBallState) {
//            case GREEN:
//                theGamepad2.setLedColor(TBDGamepad.Colors.GREEN);
//                break;
//            case PURPLE:
//                theGamepad2.setLedColor(TBDGamepad.Colors.PURPLE);
//                break;
//            case VACANT:
//                theGamepad2.setLedColor(TBDGamepad.Colors.OFF);
//        }
//        if (3 > robot.indexer.getBallNumber()) {
//            theGamepad1.setLedColor(TBDGamepad.Colors.YELLOW);
//        } else {
//            theGamepad1.setLedColor(TBDGamepad.Colors.RED);
//        }

        // --- Drive Controls ---
        double forward = theGamepad1.getLeftY();
        double strafe = theGamepad1.getLeftX();
        double turn = theGamepad1.getRightX();
        double speed = ThunderBot2025.DEFAULT_SPEED;
        double turnFactor = 0.7;

        if (theGamepad1.getTriggerBoolean(TBDGamepad.Trigger.RIGHT_TRIGGER)) {
            speed = ThunderBot2025.MIN_SPEED;
        } else if (theGamepad1.getTriggerBoolean(TBDGamepad.Trigger.LEFT_TRIGGER)) {
            speed = ThunderBot2025.MAX_SPEED;
        }
        if (theGamepad1.getButtonPressed(TBDGamepad.Button.LEFT_BUMPER)) {
            turnFactor = 1.0;
        }

        if (115 <= robot.runtime.seconds() && 125 > robot.runtime.seconds()) {
            if (1 == Math.ceil(robot.runtime.seconds() * 2) % 2) {
                theGamepad1.blipDriver();
                theGamepad2.blipDriver();
            }
        }

        // Note: The driver's 'Y' button is used for resetting pose.
        if (theGamepad1.getButton(TBDGamepad.Button.DPAD_UP)) {
            robot.resetHeadingAndPosition();
        }
        if (theGamepad2.getButton(TBDGamepad.Button.DPAD_UP)) {
            if (robot.resetTurret()) theGamepad2.blipDriver();
        }

        robot.drive(forward, strafe, turn * 0.7, speed, p);

        if (theGamepad1.getButtonPressed(TBDGamepad.Button.DPAD_DOWN)) {
            robot.kickstand.switchState();
        }

        // --- Launcher Controls ---
        if (theGamepad2.getButtonPressed(TBDGamepad.Button.RIGHT_STICK_BUTTON)) {
            if (LauncherFacade.AimingMode.MAIN == robot.launcher.getAimingMode()) {
                robot.launcher.setAimingMode(LauncherFacade.AimingMode.MANUAL);
            } else {
                robot.launcher.setAimingMode(LauncherFacade.AimingMode.MAIN);
            }
        }

        if (LauncherFacade.AimingMode.MANUAL == robot.launcher.getAimingMode()) {
            if (0.01 < Math.abs(Math.sqrt(Math.pow(theGamepad2.getRightX(), 2) + Math.pow(theGamepad2.getRightY(), 2)))) {
                manualAngle = Math.toDegrees(Math.atan2(theGamepad2.getRightY(), theGamepad2.getRightX()));
            }
            robot.launcher.aimToAngleInFieldSpace(manualAngle);
        } else if (LauncherFacade.AimingMode.DIRECTIONAL == robot.launcher.getAimingMode()) {
            robot.launcher.setTurretManualPower(theGamepad2.getRightX() * 0.35);
        } else if (0.01 < Math.abs(Math.sqrt(Math.pow(theGamepad2.getRightX(), 2) + Math.pow(theGamepad2.getRightY(), 2)))) {
            robot.launcher.aimToAngleInFieldSpace(Math.toDegrees(Math.atan2(theGamepad2.getRightY(), theGamepad2.getRightX())));
        } else {
            robot.launcher.aim();
        }

        if (theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_DOWN)) {
            preSpinUp = !preSpinUp;
        }

        if (theGamepad2.getTriggerBoolean(TBDGamepad.Trigger.LEFT_TRIGGER)) {
            robot.charge();
        } else {
            if (preSpinUp) {
                robot.chargeLow();
            } else {
                robot.launcher.stop();
            }
        }

        if (theGamepad2.getTriggerPressed(TBDGamepad.Trigger.RIGHT_TRIGGER)) {
            robot.launch();
        }
        if (theGamepad2.getButtonPressed(TBDGamepad.Button.Y)) {
            robot.indexer.prepSequence();
        }


        // --- Intake Controls (Stateful Latch) ---
        if (theGamepad2.getButton(TBDGamepad.Button.X) || theGamepad1.getButton(TBDGamepad.Button.X)) {
            robot.intakeStart();
        } else if (theGamepad2.getButton(TBDGamepad.Button.B) || theGamepad1.getButton(TBDGamepad.Button.B)) {
            robot.intakeStop();
        } else if (theGamepad2.getButton(TBDGamepad.Button.A) || theGamepad1.getButton(TBDGamepad.Button.A)) {
            robot.intake.spit();
        }

        if (isAutoLoading) {
            // --- AUTO-LOADING MODE ---
            // When a ball arrives in the slot we are watching, cycle to the next empty one.
//            if (slotToWatch != -1 && robot.indexer.getBallState(slotToWatch) != IndexerFacade.BallState.VACANT) {
//                robot.indexer.selectNextSlot(IndexerFacade.BallState.VACANT);
//                slotToWatch = robot.indexer.getCurrentTargetSlot();
//            }

        } else {
            // --- MANUAL INDEXER MODE ---
            if (theGamepad2.getButtonPressed(TBDGamepad.Button.LEFT_BUMPER)) {
                robot.indexer.overrideLaunching(!robot.indexer.isOverridden());
            }
            if (theGamepad2.getButton(TBDGamepad.Button.RIGHT_BUMPER)) {
                robot.launchAll();
            } else {

                // Then, check for discrete, one-shot commands.
                if (theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_LEFT)) {
                    robot.indexer.cycle(-1);
                } else if (theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_RIGHT)) {
                    robot.indexer.cycle(1);
                } else if (theGamepad2.getButton(TBDGamepad.Button.LEFT_STICK_BUTTON)) {
                    robot.indexer.adjustToThird();
                } else if (TransferFacade.State.IDLE == robot.indexer.getCurrentState() ) {

                    // If not manually spinning, send a spin(0) to allow the turnstile to auto-align.
//                    robot.indexer.spin(0);
                }
            }
        }

        robot.drive.localizer.update();


        dashboard.sendTelemetryPacket(p);
    }
}
