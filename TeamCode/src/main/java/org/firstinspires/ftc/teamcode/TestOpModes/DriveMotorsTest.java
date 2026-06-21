package org.firstinspires.ftc.teamcode.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot.Drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

@TeleOp(group = IndexerTest.TEST_TELEOP_GROUP)
@Disabled
//@Config
public class DriveMotorsTest extends OpMode {
    private TBDGamepad theGamepad1;
    private TBDGamepad theGamepad2;
    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;

    @Override
    public void init() {
        HardwareMap hwmap = hardwareMap;
        Telemetry telem = null;
        theGamepad1 = new TBDGamepad(gamepad1);
        theGamepad2 = new TBDGamepad(gamepad2);
        telemetry = new MultipleTelemetry(telem, FtcDashboard.getInstance().getTelemetry());
        leftBack = hwmap.get(DcMotorEx.class, MecanumDrive.LEFT_BACK_MOTOR);
        leftFront = hwmap.get(DcMotorEx.class, MecanumDrive.LEFT_FRONT_MOTOR);
        rightBack = hwmap.get(DcMotorEx.class, MecanumDrive.RIGHT_BACK_MOTOR);
        rightBack = hwmap.get(DcMotorEx.class, MecanumDrive.RIGHT_BACK_MOTOR);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        updateMotors(theGamepad1.gamepad.left_trigger, theGamepad1.gamepad.right_trigger,
                theGamepad2.gamepad.left_trigger, theGamepad2.gamepad.right_trigger);
    }

    public double getCurrent(DcMotorEx dcMotorEx) {
        return dcMotorEx.getCurrent(CurrentUnit.AMPS);
    }

    private void updateMotors(double leftFrontValue, double rightFrontValue,
                              double leftBackValue, double rightBackValue) {
        updateMotor(leftFront, leftFrontValue);
        updateMotor(rightFront, rightFrontValue);
        updateMotor(leftBack, leftBackValue);
        updateMotor(rightBack, rightBackValue);
    }

    private void updateMotor(DcMotorEx dcMotorEx, double power) {
        dcMotorEx.setPower(power);
        telemetry.addData(dcMotorEx + " Current", getCurrent(dcMotorEx));
    }
}
