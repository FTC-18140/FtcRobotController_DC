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
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

@TeleOp
@Config
public class Teleop_Red extends OpMode {

    public TelemetryPacket p = new TelemetryPacket(true);

    // --- Mode States ---
    private boolean isAutoLoading = false;
    private int slotToWatch = -1;

    ThunderBot2025.Alliance_Color alliance = ThunderBot2025.Alliance_Color.RED;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    private TBDGamepad theGamepad1;
    private TBDGamepad theGamepad2;

    ThunderBot2025 robot = new ThunderBot2025();
    public static double INDEXER_SPEED = 0.8;


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
    }
    public void start() {
        robot.runtime.reset();
    }

    @Override
    public void loop() {
        robot.update();
        theGamepad1.update();
        theGamepad2.update();

        // --- Drive Controls ---
        double forward = -theGamepad1.getLeftY();
        double strafe = -theGamepad1.getLeftX();
        double turn = theGamepad1.getRightX();
        double speed = ThunderBot2025.DEFAULT_SPEED;

        if(theGamepad1.getTriggerBoolean(TBDGamepad.Trigger.RIGHT_TRIGGER)){
            speed = ThunderBot2025.MIN_SPEED;
        } else if(theGamepad1.getTriggerBoolean(TBDGamepad.Trigger.LEFT_TRIGGER)){
            speed = ThunderBot2025.MAX_SPEED;
        }

        // Note: The driver's 'Y' button is used for resetting pose.
        if(theGamepad1.getButton(TBDGamepad.Button.Y)){
            robot.drive.localizer.setPose(new Pose2d(robot.drive.localizer.getPose().position, 0));
        }

        robot.drive(forward, strafe, turn * 0.7, speed, p);

        // --- Launcher Controls ---
        if(Math.abs(theGamepad2.getRightX()) > 0.01){
            robot.launcher.setTurretManualPower(-theGamepad2.getRightX());
        } else if(Math.abs(theGamepad1.getRightX()) > 0.01){
            robot.launcher.augmentedAim(-theGamepad1.getRightX() * speed * 0.75);
        } else {
            robot.launcher.aim();
        }

        if(theGamepad2.getTriggerBoolean(TBDGamepad.Trigger.LEFT_TRIGGER)){
            robot.charge();
        } else {
            robot.launcher.stop();
        }
         
        if(theGamepad2.getTriggerBoolean(TBDGamepad.Trigger.RIGHT_TRIGGER)){
            robot.flip();
        }
        if (theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_UP)){
            robot.launcher.flywheel.adjustFF(1);
        } else if (theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_DOWN)) {
            robot.launcher.flywheel.adjustFF(-1);
        } else if (theGamepad2.getButtonPressed(TBDGamepad.Button.Y)) {
            robot.launcher.flywheel.resetFF();
        }

        if (theGamepad2.getButtonPressed(TBDGamepad.Button.RIGHT_STICK_BUTTON)) {
            robot.flipperUp();
        } else if ( theGamepad2.getButtonReleased(TBDGamepad.Button.RIGHT_STICK_BUTTON)) {
            robot.flipperDown();
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
                robot.indexer.spin(-INDEXER_SPEED);
            } else if (theGamepad2.getButton(TBDGamepad.Button.RIGHT_BUMPER)) {
                robot.indexer.spin(INDEXER_SPEED);
            } else {
                // If not manually spinning, send a spin(0) to allow the turnstile to auto-align.
                //robot.indexer.cycle(0);

                // Then, check for discrete, one-shot commands.
                if(theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_LEFT)){
                    robot.indexer.cycle(1);
                } else if (theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_RIGHT)) {
                    robot.indexer.cycle(-1);
                } else if (theGamepad2.getButton(TBDGamepad.Button.LEFT_STICK_BUTTON)){
                    robot.indexer.adjustToThird();
                }
            }
        }

        robot.drive.localizer.update();
        telemetry.addData("Indexer Mode", isAutoLoading ? "AUTO-LOADING" : "MANUAL");
        telemetry.addData("position X: ", robot.drive.localizer.getPose().position.x);
        telemetry.addData("position Y: ", robot.drive.localizer.getPose().position.y);
        telemetry.addData("heading: ", Math.toDegrees(robot.drive.localizer.getPose().heading.toDouble()));
        telemetry.addData("Turret aiming mode:", robot.launcher.isUsingLimelight());

        dashboard.sendTelemetryPacket(p);
    }
}
