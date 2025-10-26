package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

@TeleOp
@Config
public class Teleop_Red extends OpMode {

    public TelemetryPacket p = new TelemetryPacket(true);
    private boolean barrel_spin = false;
    private boolean revolving = false;
    //public static boolean field_centric = true;

    String alliance = "red";

    FtcDashboard dashboard = FtcDashboard.getInstance();
    private TBDGamepad theGamepad1;
    private TBDGamepad theGamepad2;

    ThunderBot2025 robot = new ThunderBot2025();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, new Pose2d(-12, 12, 0));
        robot.launcher.color = alliance;
        robot.setColor(alliance);
        theGamepad1 = new TBDGamepad(gamepad1);
        theGamepad2 = new TBDGamepad(gamepad2);
        // Tell the driver that initialization is complete.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.drive.updatePoseEstimate();
        double forward = theGamepad1.getLeftY();
        double strafe = theGamepad1.getLeftX();
        double turn = theGamepad1.getRightX();
        double speed = 0.7;

        if(theGamepad1.getTriggerPressed(TBDGamepad.Trigger.RIGHT_TRIGGER)){
            speed = 0.3;
        } else if(theGamepad1.getTriggerPressed(TBDGamepad.Trigger.LEFT_TRIGGER)){
            speed = 1.0;
        }


        robot.update();
        if(Math.abs(theGamepad2.getRightX()) > 0.05){
            robot.launcher.aim(theGamepad2.getRightX() * 0.2);
        }else {
            robot.lockOn();
        }

//        if(robot.isFieldCentric())
//        {
//            robot.fieldCentricDrive(forward, strafe, turn, 0.7, p);
//        } else {
//            robot.robotCentricDrive(forward, strafe, turn, 0.7);
//        }

        robot.drive(forward, strafe, turn, speed, p);

        if(theGamepad2.getButton(TBDGamepad.Button.X)){
            robot.intake.intake();
        } else if (theGamepad2.getButton(TBDGamepad.Button.B)) {
            robot.intake.stop();
        } else if (theGamepad2.getButton(TBDGamepad.Button.A)) {
            robot.intake.spit();
        }


        if(theGamepad2.getTrigger(TBDGamepad.Trigger.LEFT_TRIGGER) >= 0.1){
            robot.launcher.shoot(robot.drive.localizer.getPose());
        } else {
            robot.launcher.stop();
        }

        //Flipper / launch controls
        if(theGamepad2.getTriggerBoolean(TBDGamepad.Trigger.RIGHT_TRIGGER) && !revolving){
            robot.indexer.flip();
            if(!barrel_spin){
                robot.indexer.cycle(-1);
                barrel_spin = true;
            }
        } else {
            //Prevents indexer from interfering with Flipper
            robot.indexer.unflip();

            if(theGamepad2.getButton(TBDGamepad.Button.LEFT_STICK_BUTTON)){
                robot.indexer.adjustToThird();
                revolving = true;
            }else if(theGamepad2.getButton(TBDGamepad.Button.LEFT_BUMPER)){
                robot.indexer.spin(-0.2);
                revolving = true;
            } else if (theGamepad2.getButton(TBDGamepad.Button.RIGHT_BUMPER)) {
                robot.indexer.spin(0.2);
                revolving = true;
            } else if(theGamepad2.getButton(TBDGamepad.Button.DPAD_LEFT)){
                if(!barrel_spin){
                    robot.indexer.cycle(-1);
                    barrel_spin = true;
                }
            } else if (theGamepad2.getButton(TBDGamepad.Button.DPAD_RIGHT)) {
                if(!barrel_spin){
                    robot.indexer.cycle(1);
                    barrel_spin = true;
                }
            } else {
                revolving = !robot.indexer.update();
                barrel_spin = false;
            }
        }

        telemetry.addData("position X: ", robot.drive.localizer.getPose().position.x);
        telemetry.addData("position Y: ", robot.drive.localizer.getPose().position.y);
        telemetry.addData("heading: ", Math.toDegrees(robot.drive.localizer.getPose().heading.toDouble()));
        telemetry.addData("rpm: ", robot.launcher.avgRpm * robot.launcher.timeDifference);
        telemetry.addData("flipper: ", robot.indexer.getFlipperPos());
        telemetry.addData("goal distance: ", robot.launcher.goalDistance(robot.drive.localizer.getPose()));


        dashboard.sendTelemetryPacket(p);
    }
}
