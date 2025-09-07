package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TBDGamepad;

@TeleOp
public class LauncherTest extends OpMode {
    private TBDGamepad theGamepad1;
    DcMotor launcher = null;

    @Override
    public void init() {
        launcher = hardwareMap.dcMotor.get("launcher");

        theGamepad1 = new TBDGamepad(gamepad1);
    }

    @Override
    public void loop() {
        if(theGamepad1.getButton(TBDGamepad.Button.X)){
            launcher.setPower(1.0);
        } else if (theGamepad1.getButton(TBDGamepad.Button.B)) {
            launcher.setPower(0.0);
        }
    }
}
