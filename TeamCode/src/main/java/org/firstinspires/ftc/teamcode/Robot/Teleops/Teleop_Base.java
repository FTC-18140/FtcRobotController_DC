package org.firstinspires.ftc.teamcode.Robot.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2026;
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

@Config
public abstract class Teleop_Base extends OpMode
{
    protected abstract ThunderBot2026.Alliance_Color getAlliance();

    public static final String MATCH_TELEOP_GROUP = "AAAMatchTeleops";
    public TelemetryPacket p = new TelemetryPacket(true);

    ThunderBot2026.Alliance_Color alliance = ThunderBot2026.Alliance_Color.BLUE;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    private TBDGamepad theGamepad1 = null;
    private TBDGamepad theGamepad2 = null;

    ThunderBot2026 robot = new ThunderBot2026();
    private boolean preSpinUp = true;
    double manualAngle;
    ElapsedTime loopTimer = new ElapsedTime( ElapsedTime.Resolution.MILLISECONDS);

    /**
     * Performs hardware mapping, initializes robot subsystems,
     * and sets default gamepad configurations.
     */
    @Override
    public void init()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, null);

        theGamepad1 = new TBDGamepad(gamepad1);
        theGamepad2 = new TBDGamepad(gamepad2);
        theGamepad1.setLedColor(TBDGamepad.Colors.ORANGE);
        theGamepad2.setLedColor(TBDGamepad.Colors.BLUE);
        theGamepad1.init(telemetry);
        theGamepad2.init(telemetry);
        // Tell the driver that initialization is complete.

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        theGamepad1.setLedColor(TBDGamepad.Colors.YELLOW);
    }

    public void start()
    {
        robot.runtime.reset();
        robot.setColor(getAlliance());
        loopTimer.reset();
    }

    /**
     * Main control loop executed repeatedly while the OpMode is running.
     * Logic is partitioned into functional blocks: Drive, Intake, Launcher, and Indexer.
     */
    @Override
    public void loop()
    {
        telemetry.addData("Loop Time", loopTimer.seconds());
        loopTimer.reset();
        /*
         * GAMEPAD 1 (DRIVER) CONTROLS:
         * --------------------------------------------------------------------------------------
         * ACTION                 | INPUT               | DESCRIPTION
         * -----------------------|---------------------|---------------------------------------
         * Drive Translation      | Left Stick (Y, X)   | Field-centric movement (Forward/Strafe)
         * Drive Rotation         | Right Stick X       | Standard rotation (Turn)
         * -----------------------|---------------------|---------------------------------------
         * Turbo Mode             | Left Trigger        | Increases drive speed to MAX_SPEED
         * Slow Mode              | Right Trigger       | Decreases drive speed to MIN_SPEED
         * High-Sens Turn         | Left Bumper         | Increases rotation sensitivity to 100%
         * -----------------------|---------------------|---------------------------------------
         * Reset Pose             | D-Pad Up            | Resets field-centric heading/position
         * -----------------------|---------------------|---------------------------------------
         * Intake Start           | Button X            | Activates intake (shared with GP2)
         * Intake Stop            | Button B            | Stops intake (shared with GP2)
         * Intake Reverse (Spit)  | Button A            | Reverses intake (shared with GP2)
         * --------------------------------------------------------------------------------------
         */

        /*
         * GAMEPAD 2 (OPERATOR) CONTROLS:
         * --------------------------------------------------------------------------------------
         * ACTION                 | INPUT               | DESCRIPTION
         * -----------------------|---------------------|---------------------------------------
         * Toggle Aiming Mode     | Right Stick Button  | Switches between Main (Auto) and Manual
         * Manual Turret Control  | Right Stick (Y, X)  | Sets field-angle or power based on mode
         * Home Turret            | D-Pad Up            | Resets turret to zero/home position
         * -----------------------|---------------------|---------------------------------------
         * Toggle Idle Flywheel   | D-Pad Down          | Toggles low-speed "Pre-Spin" state
         * Charge Flywheel        | Left Trigger        | Full power spin-up for firing
         * Launch Single          | Right Trigger       | Launches one game piece
         * -----------------------|---------------------|---------------------------------------
         * Prep Sequence          | Button Y            | Runs indexer preparation state machine
         * Launch All             | Right Bumper        | Rapidly launches all available pieces
         * Toggle Launch Override | Left Bumper         | Enables/Disables manual launch control
         * Cycle Indexer          | D-Pad Left/Right    | Manually moves turnstile slots
         * Indexer Slot 3         | Left Stick Button   | Adjusts indexer to third position
         * -----------------------|---------------------|---------------------------------------
         * Intake Controls        | Buttons X, B, A     | Shared with GP1 (Start, Stop, Spit)
         * --------------------------------------------------------------------------------------
         */

        // --- 1. CORE UPDATES ---
        // Refresh robot hardware states and process gamepad debounce logic.
        robot.update();
        theGamepad1.update();
        theGamepad2.update();

        // --- 2. DRIVER NOTIFICATIONS (END-GAME) ---
        // Pulse controllers during the last 30 seconds of the match to alert drivers
        if (robot.runtime.seconds() >= 115 && robot.runtime.seconds() < 125)
        {
            if (Math.ceil(robot.runtime.seconds() * 2) % 2 == 1)
            {
                theGamepad1.blipDriver();
                theGamepad2.blipDriver();
            }
        }

        // --- 3. DRIVE CONTROLS (GAMEPAD 1) ---
        double forward = theGamepad1.getLeftY();
        double strafe = theGamepad1.getLeftX();
        double turn = theGamepad1.getRightX();
        double speed = ThunderBot2026.DEFAULT_SPEED;
        double turnFactor = 0.7;

        // Speed modifiers
        if (theGamepad1.getTriggerBoolean(TBDGamepad.Trigger.RIGHT_TRIGGER))
        {
            speed = ThunderBot2026.MIN_SPEED; // Slow mode
        }
        else if (theGamepad1.getTriggerBoolean(TBDGamepad.Trigger.LEFT_TRIGGER))
        {
            speed = ThunderBot2026.MAX_SPEED; // Turbo mode
        }

        // Handling turn sensitivity
        if (theGamepad1.getButtonPressed(TBDGamepad.Button.LEFT_BUMPER))
        {
            turnFactor = 1.0;
        }

        // Field-centric reset and utilities
        if (theGamepad1.getButton(TBDGamepad.Button.DPAD_UP))
        {
            robot.resetHeadingAndPosition();
        }

        // Apply movement to drivetrain
        robot.drive(forward, strafe, turn * turnFactor, speed);

        // --- 4. INTAKE CONTROLS (SHARED) ---
        // Both Driver and Operator can control the intake
        if (theGamepad2.getButton(TBDGamepad.Button.X) || theGamepad1.getButton(TBDGamepad.Button.X))
        {
            robot.intakeStart();
        }
        else if (theGamepad2.getButton(TBDGamepad.Button.B) || theGamepad1.getButton(TBDGamepad.Button.B))
        {
            robot.intakeStop();
        }
        else if (theGamepad2.getButton(TBDGamepad.Button.A) || theGamepad1.getButton(TBDGamepad.Button.A))
        {
            robot.intake.spit();
        }

        // --- 5. LAUNCHER & TURRET CONTROLS (GAMEPAD 2) ---

        // Turret Homing/Reset
        if (theGamepad2.getButton(TBDGamepad.Button.DPAD_UP))
        {
            if (robot.resetTurret())
            {
                theGamepad2.blipDriver();
            }
        }

        // Aiming Mode Selection: Toggle between MAIN (Auto) and MANUAL
        if (theGamepad2.getButtonPressed(TBDGamepad.Button.RIGHT_STICK_BUTTON))
        {
            robot.launcher.toggleAim();
        }

        // Handle Manual Aiming Inputs ONLY if not in Auto (MAIN) mode

//        if (currentMode == LauncherFacade.AimingMode.MANUAL)
//        {
        // Calculate stick magnitude to determine if the driver is actively aiming
        double stickMag = Math.sqrt(Math.pow(theGamepad2.getRightX(), 2) + Math.pow(theGamepad2.getRightY(), 2));

        if (stickMag > 0.1)
        {
            // Set the desired field-centric angle based on stick direction
            manualAngle = Math.toDegrees(Math.atan2(theGamepad2.getRightY(), theGamepad2.getRightX()));
            telemetry.addData("Manual Angle", manualAngle);
            robot.launcher.moveAimingTarget(manualAngle);
        }
//            else
//            {
//                // If stick is released, hold current position to prevent drifting
//                robot.launcher.holdTurretPosition();
//            }
//        }
//        else if (currentMode == LauncherFacade.AimingMode.DIRECTIONAL)
//        {
//            // Use Right X for raw power override
//            robot.launcher.setTurretManualPower(theGamepad2.getRightX() * 0.35);
//        }
        // NOTE: If mode is MAIN, LauncherFacade.update() handles aim() automatically.
        // No manual call to robot.launcher.aim() is needed here anymore.

        // Toggle idle state
        if (theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_DOWN))
        {
            preSpinUp = !preSpinUp;
        }

        // Flywheel Power Management
        // 1. Trigger held: High power tracking the goal distance
        if (theGamepad2.getTriggerBoolean(TBDGamepad.Trigger.LEFT_TRIGGER))
        {
            robot.charge(); // Mode -> DISTANCE. Targets update every loop as robot moves.
        }
        else if (preSpinUp)
        {
            robot.chargeLow(); // Mode -> STATIC. Steady 1800 RPM.
        }
        else
        {
            robot.chargeStop(); // Mode -> OFF. Motors float.
        }

        // Shooting Commands
        if (theGamepad2.getTriggerPressed(TBDGamepad.Trigger.RIGHT_TRIGGER))
        {
            robot.launch();
        }

//        // --- 6. INDEXER CONTROLS (GAMEPAD 2) ---
//        if (theGamepad2.getButtonPressed(TBDGamepad.Button.Y))
//        {
//            robot.indexer.prepSequence();
//        }

        // --- MANUAL INDEXER MODE ---
        if (theGamepad2.getButton(TBDGamepad.Button.LEFT_BUMPER))
        {
            robot.indexer.overrideLaunching(!robot.indexer.isOverridden());
        }

        if (theGamepad2.getButton(TBDGamepad.Button.RIGHT_BUMPER))
        {
            robot.launchAll();
            theGamepad2.blipDriver();
        }
        else
        {
            // Discrete manual movements
            if (theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_LEFT))
            {
                robot.indexer.cycle(-1);
            }
            else if (theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_RIGHT))
            {
                robot.indexer.cycle(1);
            }
            else if (theGamepad2.getButton(TBDGamepad.Button.LEFT_STICK_BUTTON))
            {
                robot.indexer.adjustToThird();
            }
        }

        // --- 7. FINAL UPDATES & TELEMETRY ---
        robot.drive.localizer.update();
        dashboard.sendTelemetryPacket(p);

    }

}
