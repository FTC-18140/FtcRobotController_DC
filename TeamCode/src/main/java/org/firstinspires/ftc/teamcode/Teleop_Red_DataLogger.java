package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.IndexerFacade;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;
import org.firstinspires.ftc.teamcode.Utilities.DataLogger;
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@TeleOp(name = "Teleop Red with Datalogger")
@Config
public class Teleop_Red_DataLogger extends OpMode {

    public TelemetryPacket p = new TelemetryPacket(true);

    // --- Mode States -- -
    private boolean isAutoLoading = false;
    private int slotToWatch = -1;

    ThunderBot2025.Alliance_Color alliance = ThunderBot2025.Alliance_Color.RED;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    private TBDGamepad theGamepad1;
    private TBDGamepad theGamepad2;

    ThunderBot2025 robot = new ThunderBot2025();

    private DataLogger logger;
    private boolean isLogging = false;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, null);
        robot.setColor(alliance);

        theGamepad1 = new TBDGamepad(gamepad1);
        theGamepad2 = new TBDGamepad(gamepad2);
        // Tell the driver that initialization is complete.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        logger = new DataLogger();
    }

    private void startLogging() {
        try {
            SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault());
            String timestamp = sdf.format(new Date());
            String fileName = "TeleopData_" + timestamp;
            logger.openFile(fileName);
            isLogging = true;
            telemetry.addData("Logging", "Started: " + fileName);

            // Add header
            logger.addField("Limelight.x");
            logger.addField("Turret.targetAngle");
            logger.addField("Turret.currentPosition");
            logger.addField("Turret.P");
            logger.addField("Turret.I");
            logger.addField("Turret.D");
            logger.addField("Turret.seekingPower");
            logger.addField("Localizer.pose.x");
            logger.addField("Localizer.pose.y");
            logger.addField("Localizer.pose.heading");
            logger.newLine();
        } catch (IOException e) {
            isLogging = false;
            telemetry.addData("Logging", "Error starting: " + e.getMessage());
        }
    }

    private void stopLogging() {
        if (isLogging) {
            logger.closeDataLogger();
            isLogging = false;
            telemetry.addData("Logging", "Stopped");
        }
    }

    @Override
    public void loop() {
        robot.update();
        theGamepad1.update();
        theGamepad2.update();

        if (theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_UP)) {
            if (!isLogging) {
                startLogging();
            }
        }

        if (theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_DOWN)) {
            if (isLogging) {
                stopLogging();
            }
        }

        // --- Drive Controls ---
        double forward = theGamepad1.getLeftY();
        double strafe = theGamepad1.getLeftX();
        double turn = theGamepad1.getRightX();
        double speed = 0.7;

        if(theGamepad1.getTriggerBoolean(TBDGamepad.Trigger.RIGHT_TRIGGER)){
            speed = 0.3;
        } else if(theGamepad1.getTriggerBoolean(TBDGamepad.Trigger.LEFT_TRIGGER)){
            speed = 1.0;
        }

        // Note: The driver's 'Y' button is used for resetting pose.
        if(theGamepad1.getButton(TBDGamepad.Button.Y)){
            robot.drive.localizer.setPose(new Pose2d(robot.drive.localizer.getPose().position, 0));
        }

        robot.drive(forward, strafe, turn * 0.7, speed, p);

        // --- Launcher Controls ---
        if(Math.abs(theGamepad2.getRightX()) > 0.01){
            robot.launcher.augmentedAim(-1.2 * theGamepad2.getRightX() + theGamepad1.getRightX() * speed * 0.75);
        } else if(Math.abs(theGamepad1.getRightX()) > 0.01){
            robot.launcher.augmentedAim(theGamepad1.getRightX() * speed * 0.75);
        } else {
            robot.launcher.aim();
        }

        if(theGamepad2.getTriggerBoolean(TBDGamepad.Trigger.LEFT_TRIGGER)){
            robot.charge();
        } else {
            robot.launcher.stop();
        }

        if(theGamepad2.getTriggerBoolean(TBDGamepad.Trigger.RIGHT_TRIGGER)){
            robot.indexer.flip();
        }

        // --- Intake Controls (Stateful Latch) ---
        if(theGamepad2.getButton(TBDGamepad.Button.X) || theGamepad1.getButton(TBDGamepad.Button.X)){
            robot.intake.intake();
        } else if (theGamepad2.getButton(TBDGamepad.Button.B) || theGamepad1.getButton(TBDGamepad.Button.B)) {
            robot.intake.stop();
        } else if (theGamepad2.getButton(TBDGamepad.Button.A) || theGamepad1.getButton(TBDGamepad.Button.A)) {
            robot.intake.spit();
        }

        // --- Indexer Mode Selection & Logic ---
        // 'Y' on Gamepad 2 toggles auto-loading mode.
        if (theGamepad2.getButtonPressed(TBDGamepad.Button.Y)) {
            isAutoLoading = !isAutoLoading;
            // When we enter the mode, find the first target slot.
            if (isAutoLoading) {
                robot.indexer.selectNextSlot(IndexerFacade.BallState.VACANT);
                slotToWatch = robot.indexer.getCurrentTargetSlot();
            }
        }

        // --- Manual Overrides to CANCEL Auto-Loading ---
        // Driver can cancel by spitting.
        if (theGamepad1.getButtonPressed(TBDGamepad.Button.A)) {
            isAutoLoading = false;
        }
        // Operator can cancel by pressing any manual indexer button.
        if (theGamepad2.getButton(TBDGamepad.Button.LEFT_BUMPER) ||
            theGamepad2.getButton(TBDGamepad.Button.RIGHT_BUMPER) ||
            theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_LEFT) ||
            theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_RIGHT) ||
            theGamepad2.getButtonPressed(TBDGamepad.Button.LEFT_STICK_BUTTON)) {
            isAutoLoading = false;
        }

        if (isAutoLoading) {
            // --- AUTO-LOADING MODE ---
            // When a ball arrives in the slot we are watching, cycle to the next empty one.
            if (slotToWatch != -1 && robot.indexer.getBallState(slotToWatch) != IndexerFacade.BallState.VACANT) {
                robot.indexer.selectNextSlot(IndexerFacade.BallState.VACANT);
                slotToWatch = robot.indexer.getCurrentTargetSlot();
            }
        } else {
            // --- MANUAL INDEXER MODE ---
            if(theGamepad2.getButton(TBDGamepad.Button.LEFT_BUMPER)){
                robot.indexer.spin(-0.2);
            } else if (theGamepad2.getButton(TBDGamepad.Button.RIGHT_BUMPER)) {
                robot.indexer.spin(0.2);
            } else {
                // If not manually spinning, send a spin(0) to allow the turnstile to auto-align.
                robot.indexer.cycle(0);

                // Then, check for discrete, one-shot commands.
                if(theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_LEFT)){
                    robot.indexer.cycle(-1);
                } else if (theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_RIGHT)) {
                    robot.indexer.cycle(1);
                } else if (theGamepad2.getButtonPressed(TBDGamepad.Button.LEFT_STICK_BUTTON)){
                    robot.indexer.adjustToThird();
                }
            }
        }

        if (isLogging) {
            robot.logData(logger);
            logger.newLine(); // This will flush the line buffer and add timestamps
        }

        robot.drive.localizer.update();
        telemetry.addData("Indexer Mode", isAutoLoading ? "AUTO-LOADING" : "MANUAL");
        telemetry.addData("position X: ", robot.drive.localizer.getPose().position.x);
        telemetry.addData("position Y: ", robot.drive.localizer.getPose().position.y);
        telemetry.addData("heading: ", Math.toDegrees(robot.drive.localizer.getPose().heading.toDouble()));

        dashboard.sendTelemetryPacket(p);
    }

    @Override
    public void stop() {
        stopLogging();
    }
}
