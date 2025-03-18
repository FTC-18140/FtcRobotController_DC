package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Auto.AutoPositions;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.IntakeClaw;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot_Claw_2024;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "Teleop")
public class Teleop_ClawTest_NoTimer extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private ThunderBot_Claw_2024 robot = new ThunderBot_Claw_2024();
    private TBDGamepad theGamepad1;
    private TBDGamepad theGamepad2;

    public double wristPos;
    public double clawPos;
    public double spinPos;
    public boolean turning = false;
    int armTarget = 0;
    public double liftServoPos;

    public double liftPower = 0;

    private ElapsedTime teleopTimer = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry, 0);
        robot.intake.armOffset = Intake.ARM_MAX_HORIZONTAL;
        liftServoPos  = robot.lift.LIFT_SERVO_MAX;
        robot.drive.pose = new Pose2d(AutoPositions.Positions.START_LEFT.position, Math.toRadians(45));
        wristPos = robot.intake.WRIST_INIT;
        clawPos = robot.intake.clawPos;
        spinPos = 0.0;
        theGamepad1 = new TBDGamepad(gamepad1);
        theGamepad2 = new TBDGamepad(gamepad2);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void start() {
        robot.led.ledTimer.reset();
        robot.lift.lift_timer.reset();
        robot.intake.start();
        robot.intake.setToExtended();
        teleopTimer.reset();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads
        if(theGamepad1.getButton(TBDGamepad.Button.DPAD_LEFT)){
            if(!turning) {
                runningActions.add(robot.drive.actionBuilder(robot.drive.pose).turnTo(Math.toRadians(45)).build());
                turning = true;
            }
        }else if(theGamepad1.getButton(TBDGamepad.Button.DPAD_UP)){
            if(!turning) {
                runningActions.add(robot.drive.actionBuilder(robot.drive.pose).turnTo(Math.toRadians(90)).build());
                turning = true;
            }
        }else if(theGamepad1.getButton(TBDGamepad.Button.DPAD_DOWN)){
            if(!turning) {
                runningActions.add(robot.drive.actionBuilder(robot.drive.pose).turnTo(Math.toRadians(-90)).build());
                turning = true;
            }
        }else{
            turning = false;
        }

        double forward = theGamepad1.getLeftY();
        double strafe = theGamepad1.getLeftX();
        double turn  = theGamepad1.getRightX();
        double slow = 0.7;
        double armSlow = 1;

        //robot.intake.armTo(armTarget);
        robot.intake.update();
        robot.lift.update();
//        robot.lift.leftServo.setPosition(liftServoPos);
//        robot.lift.rightServo.setPosition(liftServoPos);

        if(gamepad1.a){
            //robot.lift.offsetPos = 0;
        }

        if(theGamepad1.getTrigger(TBDGamepad.Trigger.LEFT_TRIGGER)>0.1){
            slow = 1.0;
            theGamepad1.blipDriver();
        }
        if(theGamepad1.getTrigger(TBDGamepad.Trigger.RIGHT_TRIGGER)>0.1){
            slow = 0.3;
        }
        if(theGamepad2.getTrigger(TBDGamepad.Trigger.LEFT_TRIGGER)>0.1){
            armSlow = 0.4;
        }

        if((teleopTimer.seconds() > 105 && Math.sin(2*teleopTimer.seconds()) > 0) || teleopTimer.seconds() > 120)
        {
            // robot.led.setToColor("orange");
        }
        else if (slow == 1.0)
        {
            robot.led.setToColor("rainbow");
        }
        else
        {
            robot.led.setToColor(robot.intake.getCalculatedColor());
        }

        if(theGamepad2.getButton(TBDGamepad.Button.LEFT_STICK_BUTTON)){
//            robot.intake.armOverride = true;
//            robot.intake.armOffset = robot.intake.armPos;
            robot.intake.overRideArmPos(true);
        }else{
//            robot.intake.armOverride = false;
            robot.intake.overRideArmPos(false);
        }




        if(theGamepad1.getButton(TBDGamepad.Button.B)){
            robot.lift.hang();
        }else if(theGamepad1.getButton(TBDGamepad.Button.A)){
            robot.lift.moveToMin();
//            if (liftServoPos-0.01 > 0){
//                liftServoPos -= 0.01;
//            }
            liftPower = 0;
        }

        if(theGamepad1.getButton(TBDGamepad.Button.RIGHT_BUMPER)){
            //robot.lift.lift_target = 0;
            strafe = -robot.colorOffsetX();
            forward = -robot.colorOffsetY();
//            turn = -robot.specimenAngleSmooth(false);
        }else if(theGamepad1.getButton(TBDGamepad.Button.LEFT_BUMPER)){
            //robot.lift.lift_target = robot.lift.LIFT_MAX;
            strafe = -robot.colorOffsetX();
            forward = -robot.colorOffsetY();
//            turn = -robot.specimenAngleSmooth(false);
            //telemetry.addData("AprilTag offset X: ", robot.specimenOffsetX());
        }

        //Elbow controls
        clawPos = robot.intake.clawPos;
        if(theGamepad2.getTrigger(TBDGamepad.Trigger.RIGHT_TRIGGER) > 0.1){
            if(theGamepad2.getButton(TBDGamepad.Button.Y)){
                robot.intake.preset(IntakeClaw.Positions.HIGH_CHAMBER_SCORING);
                armTarget = (int)IntakeClaw.Positions.HIGH_CHAMBER_SCORING.armPos;
                wristPos = IntakeClaw.Positions.HIGH_CHAMBER_SCORING.wristPos;
            }
            else if(theGamepad2.getButton(TBDGamepad.Button.A)){
                robot.intake.preset(IntakeClaw.Positions.INTAKE_SPECIMEN);
                armTarget = (int)IntakeClaw.Positions.INTAKE_SPECIMEN.armPos;
                wristPos = IntakeClaw.Positions.INTAKE_SPECIMEN.wristPos;
            }
            else if(theGamepad2.getButton(TBDGamepad.Button.DPAD_LEFT)){
                robot.intake.clawMove(0);
            }
            else if(theGamepad2.getButton(TBDGamepad.Button.DPAD_RIGHT)){
                robot.intake.clawMove(IntakeClaw.CLAW_OPEN);
            }
            else if(theGamepad2.getButton(TBDGamepad.Button.DPAD_UP)){
                wristPos = robot.intake.wristPos;
                wristPos -= 0.02*armSlow;
            }
            else if (theGamepad2.getButton(TBDGamepad.Button.DPAD_DOWN)) {
                robot.intake.preset(IntakeClaw.Positions.INTAKE_SPECIMEN);
                armTarget = (int)IntakeClaw.Positions.INTAKE_SPECIMEN.armPos;
                wristPos = IntakeClaw.Positions.INTAKE_SPECIMEN.wristPos;
            }
        } else {
            if(gamepad2.dpad_up){
                wristPos = robot.intake.wristPos;
                wristPos -= 0.02*armSlow;
            }
            else if(gamepad2.dpad_down){
                wristPos = IntakeClaw.WRIST_MAX;
                robot.intake.pivotTo(IntakeClaw.PIVOT_INIT);
            }

            if (theGamepad2.getButton(TBDGamepad.Button.Y)) {
                if(theGamepad2.getTriggerBoolean(TBDGamepad.Trigger.LEFT_TRIGGER)){
                    robot.intake.preset(IntakeClaw.Positions.LOW_BASKET);
                    armTarget = (int) IntakeClaw.Positions.LOW_BASKET.armPos;
                    wristPos = IntakeClaw.Positions.LOW_BASKET.wristPos;
                } else {
                    robot.intake.preset(IntakeClaw.Positions.HIGH_BASKET);
                    armTarget = (int) IntakeClaw.Positions.HIGH_BASKET.armPos;
                    wristPos = IntakeClaw.Positions.HIGH_BASKET.wristPos;
                }
            } else if (theGamepad2.getButton(TBDGamepad.Button.A)) {
                robot.intake.preset(IntakeClaw.Positions.READY_TO_INTAKE);
                armTarget = (int)IntakeClaw.Positions.READY_TO_INTAKE.armPos;
                wristPos = IntakeClaw.Positions.READY_TO_INTAKE.wristPos;
            }
            if (theGamepad2.getButton(TBDGamepad.Button.DPAD_LEFT)) {
                robot.intake.pivotTo(1.0);
            } else if (theGamepad2.getButton(TBDGamepad.Button.DPAD_RIGHT)) {
                robot.intake.pivotTo(0.0);
            }
        }
        // Arm controls
        if(theGamepad2.getButton(TBDGamepad.Button.X)){
            robot.intake.armUp(0.3*armSlow);
            //armTarget += (int)(1 * armSlow);
            //robot.intake.armTo(armTarget);
        }
        else if(theGamepad2.getButton(TBDGamepad.Button.B)){
            robot.intake.armDown(-0.65*(armSlow*1.5));
            //robot.intake.armTo
            //armTarget -= (int)(1 * armSlow);
            //robot.intake.armTo(armTarget);
        }
        else{
            if(robot.intake.armTo == 0){
                robot.intake.armStop();
            }
        }

        if(theGamepad2.getButton(TBDGamepad.Button.LEFT_BUMPER)){
            robot.intake.clawMove(IntakeClaw.CLAW_CLOSE);
            telemetry.addData("intaking",0);
        } else if (theGamepad2.getButton(TBDGamepad.Button.RIGHT_BUMPER)) {
            robot.intake.clawMove(IntakeClaw.CLAW_OPEN);
            telemetry.addData("outaking",0);
        }else{
            //robot.intake.spinStop();
        }

        if(!theGamepad2.getButton(TBDGamepad.Button.LEFT_STICK_BUTTON)){
            armTarget = (int)Range.clip(armTarget, robot.intake.ARM_MIN, Intake.ARM_MAX);
        }
        wristPos = Range.clip(wristPos, IntakeClaw.WRIST_MIN, IntakeClaw.WRIST_MAX);
        robot.intake.wristMove(wristPos);

        //robot.lift.moveLift(liftPower);

        // Send calculated power to wheels
        if (!turning){
            robot.joystickDrive(forward, strafe, turn * 0.8 * slow, slow);
        }



        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if(action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

//        telemetry.addData("left lift servo: ",robot.lift.getLeftServoPos());
//        telemetry.addData("right lift servo: ",robot.lift.getRightServoPos());
//        telemetry.addData("left motor position: ", robot.lift.getLiftPosL());
//        telemetry.addData("right motor position: ", robot.lift.getLiftPosR());
//        telemetry.addData("lift target position: ", robot.lift.lift_target);
        telemetry.addData("time: ", teleopTimer.seconds());

        dash.sendTelemetryPacket(packet);
    }
}
