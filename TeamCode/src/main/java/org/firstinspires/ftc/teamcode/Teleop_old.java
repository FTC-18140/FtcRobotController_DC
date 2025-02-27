/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2024;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="Teleop", group="Teleop")
public class Teleop_old extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ThunderBot2024 robot = new ThunderBot2024();
    private TBDGamepad theGamepad1;
    private TBDGamepad theGamepad2;

    public double wristPos;
    public double clawPos;
    public double spinPos;

    public double liftPower = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry, 2500);
        wristPos = robot.intake.WRIST_INIT;
        clawPos = robot.intake.clawPos;
        spinPos = 0.0;
        theGamepad1 = new TBDGamepad( gamepad1);
        theGamepad2 = new TBDGamepad(gamepad2);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        robot.led.ledTimer.reset();
        robot.intake.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to forward straight.
        double forward = theGamepad1.getLeftY();
        double strafe = theGamepad1.getLeftX();
        double turn  = theGamepad1.getRightX();
        double slow = 0.7;
        double armSlow = 1;

        robot.intake.update();
        robot.led.setToColor(robot.intake.getCalculatedColor());

        if(gamepad1.a){
            robot.lift.offsetPos = 0;
        }

        if(theGamepad1.getTrigger(TBDGamepad.Trigger.RIGHT_TRIGGER)>0.1){
            slow = 1.0;
            theGamepad1.blipDriver();
        };
        if(theGamepad1.getTrigger(TBDGamepad.Trigger.LEFT_TRIGGER)>0.1){
            slow = 0.3;
        }
        if(theGamepad2.getTrigger(TBDGamepad.Trigger.LEFT_TRIGGER)>0.1){
            armSlow = 0.4;
        }

        if(gamepad1.dpad_up){
            liftPower = 1;
        }else if(gamepad1.dpad_down){
            liftPower = -1;
        }else if(gamepad1.dpad_left){
            liftPower = 0;
        }


//        else if (gamepad2.dpad_left){
//            wristPos = 0.65;
//        }
//        else if(gamepad2.dpad_right){
//            wristPos = 0;
//        }
        //Elbow controls
        clawPos = robot.intake.clawPos;
        if(theGamepad2.getTrigger(TBDGamepad.Trigger.RIGHT_TRIGGER) > 0.1){
            if(theGamepad2.getButton(TBDGamepad.Button.Y)){
                robot.intake.elbowUp(1.5*armSlow);
            }
            else if(theGamepad2.getButton(TBDGamepad.Button.A)){
                robot.intake.elbowDown(1.5*armSlow);
            }
            else if(theGamepad2.getButton(TBDGamepad.Button.DPAD_LEFT)){
                robot.intake.clawMove(0);
            }
            else if(theGamepad2.getButton(TBDGamepad.Button.DPAD_RIGHT)){
                robot.intake.clawMove(Intake.CLAW_MAX);
            }
            else if (theGamepad2.getButton(TBDGamepad.Button.DPAD_DOWN)) {
                robot.intake.preset(Intake.Positions.INTAKE_SPECIMEN);
                wristPos = Intake.Positions.INTAKE_SPECIMEN.wristPos;
            }
        } else {
            if(gamepad2.dpad_up){
                wristPos = robot.intake.wristPos;
                wristPos -= 0.03*armSlow;
            }
            else if(gamepad2.dpad_down){
                wristPos = robot.intake.wristPos;
                wristPos += 0.03*armSlow;
            }

            if (theGamepad2.getButton(TBDGamepad.Button.Y)) {
                robot.intake.preset(Intake.Positions.HIGH_BASKET);
                wristPos = Intake.Positions.HIGH_BASKET.wristPos;
            } else if (theGamepad2.getButton(TBDGamepad.Button.A)) {
                robot.intake.preset(Intake.Positions.READY_TO_INTAKE);
                wristPos = Intake.Positions.READY_TO_INTAKE.wristPos;
            } else if (theGamepad2.getButton(TBDGamepad.Button.DPAD_LEFT)) {
                robot.intake.preset(Intake.Positions.LOW_BASKET);
                wristPos = Intake.Positions.LOW_BASKET.wristPos;
            } else if (theGamepad2.getButton(TBDGamepad.Button.DPAD_RIGHT)) {
                robot.intake.preset(Intake.Positions.HIGH_CHAMBER);
                wristPos = Intake.Positions.HIGH_CHAMBER.wristPos;
            }
        }
        // Arm controls
        if(theGamepad2.getButton(TBDGamepad.Button.X)){
            robot.intake.armUp(0.4*armSlow);
        }
        else if(theGamepad2.getButton(TBDGamepad.Button.B)){
            robot.intake.armDown(-0.8*(armSlow*1.5));
        }
        else{
            if(robot.intake.aDouble == 0){
                robot.intake.armStop();
            }
        }

        if(theGamepad2.getButton(TBDGamepad.Button.LEFT_BUMPER)){
            robot.intake.spin(1);
            telemetry.addData("intaking",0);
        } else if (theGamepad2.getButton(TBDGamepad.Button.RIGHT_BUMPER)) {
            robot.intake.spin(-0.25);
            telemetry.addData("outaking",0);
        }else{
            robot.intake.spinStop();
        }

        wristPos = Range.clip(wristPos, robot.intake.WRIST_MIN, robot.intake.WRIST_MAX);
        robot.intake.wristMove(wristPos);

        //robot.lift.moveLift(liftPower);

        // Send calculated power to wheels

        if(theGamepad1.getButton(TBDGamepad.Button.B)){

        }else {
            robot.joystickDrive(forward, strafe, turn * 0.8 * slow, slow);
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Joystick Commands", "forward (%.2f), strafe (%.2f), turn (%.2f)", forward, strafe, turn);
        telemetry.addData("servo wrist position: ", robot.intake.wristPos);
        telemetry.addData("wristPos : ", wristPos);
        telemetry.addData("liftPos : ", robot.lift.getLiftPosR());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
