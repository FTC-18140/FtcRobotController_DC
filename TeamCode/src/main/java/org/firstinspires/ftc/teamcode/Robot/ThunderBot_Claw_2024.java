package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.ThunderBot2024.GearRatio.TWELVE_TO_ONE;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Pixycam.Pixy;
import org.firstinspires.ftc.teamcode.Robot.Pixycam.PixyBlock;

import java.util.ArrayList;
import java.util.List;

;

@Config
public class ThunderBot_Claw_2024
{
    public IntakeClaw intake;
    public MecanumDrive drive;
    public Lift lift;
    public LED led;
    public WebCamVision webCam;
    public LimelightVision limelight;
    public Pixy pixy;
    public ArrayList<PixyBlock> blocks;
    private PixyBlock detectedBlock;
    double heading = 0;
    long leftFrontPosition = 0;
    long rightFrontPosition = 0;
    long leftRearPosition = 0;
    long rightRearPosition = 0;
    List<LynxModule> allHubs;

    public static boolean useFieldCenteredDrive = false;
    boolean moving = false;
    public enum GearRatio
    {
        THREE_TO_ONE(2.89),
        FOUR_TO_ONE(3.61),
        FIVE_TO_ONE(5.23),
        TWELVE_TO_ONE(10.4329),
        FIFTEEN_TO_ONE(15.1147),
        SIXTEEN_TO_ONE(13.0321),
        TWENTY_TO_ONE(18.8803);
        public final double value;
        GearRatio(double theRatio) { value = theRatio; }
    }


    // converts inches to motor ticks
    static final double COUNTS_PER_MOTOR_REV = 28; // REV HD Hex motor
    static final double WHEEL_DIAMETER_CM = 9.6;  // goBilda mecanum wheels are 96mm in diameter
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * TWELVE_TO_ONE.value)
            / (WHEEL_DIAMETER_CM * Math.PI);

    private Telemetry telemetry = null;
    private long initPosition;
    private double startAngle;


    public enum Direction
    {
        LEFT,
        RIGHT;
    }

    /**
     * Constructor
     */
    public ThunderBot_Claw_2024() {}

    /**
     * Initializes the Thunderbot and connects its hardware to the HardwareMap
     *
     * @param hwMap
     * @param telem
     */
    public void init(HardwareMap hwMap, Telemetry telem, double liftOffset)
    {
        telemetry = telem;

        intake = new IntakeClaw();
        intake.init(hwMap,telem);

        lift = new Lift();
        lift.init(hwMap, telem, liftOffset);

        led = new LED();
        led.init(hwMap, telem);

        limelight = new LimelightVision();
        limelight.init(hwMap, telem);

//        webCam = new WebCamVision();
//        webCam.init(hwMap, telem);

        pixy = hwMap.get(Pixy.class, "pixy");
        detectedBlock = pixy.getBlock();
        pixy.turnOnLamps();

        drive = new MecanumDrive(hwMap, new Pose2d(0,0,0));
//  This code was somehow preventing the Odometry from updating
//        try {
//            allHubs = hwMap.getAll(LynxModule.class);
//
//            for (LynxModule module : allHubs) {
//                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            }
//        }
//        catch (Exception e) {
//            telemetry.addData("Lynx Module not initialized", 0);
//        }

    }
    public void joystickDrive(double forward, double right, double clockwise, double slow) {
        PoseVelocity2d thePose;
        Vector2d theVector;
        theVector = new Vector2d(forward, -right);
        theVector = theVector.times(slow);
        thePose = new PoseVelocity2d(theVector, -clockwise);


        PoseVelocity2d currentVel = drive.updatePoseEstimate();
        //PoseVelocity2d finalVel = new PoseVelocity2d(new Vector2d(thePose.linearVel.x+0.5*(thePose.linearVel.x-currentVel.linearVel.x),thePose.linearVel.y+0.5*(thePose.linearVel.y-currentVel.linearVel.y)), thePose.angVel+0.5*(thePose.angVel-currentVel.angVel));
        drive.setDrivePowers(thePose);


        telemetry.addData("Odometry X: ", drive.pose.position.x);
        telemetry.addData("Odometry Y: ", drive.pose.position.y);
    }
    /**
     * Make the robot drive a certain distance in a certain direction.
     * @param targetHeading
     * @param distance
     * @param power
     * @return  boolean indicating true when the move is complete
     */
    public boolean drive(double targetHeading, double distance, double power)
    {
        double currentAngle = heading;

        // Set desired angle and initial distanceMoved
        if (!moving)
        {
            startAngle = heading;
            initPosition = leftFrontPosition;
            moving = true;
        }

        double distanceMoved;
        distanceMoved = abs(leftFrontPosition - initPosition);

        double distanceMovedInCM = distanceMoved / COUNTS_PER_CM;
        telemetry.addData("distanceMoved", distanceMoved);

        double currentPower = 0.1;

        if (distanceMovedInCM <= 0.1 * distance){
            currentPower += 0.00001;
            currentPower = Range.clip(currentPower, 0.1, 1.0);
        } else if (distanceMovedInCM > 0.9 * distance){
            currentPower -= 0.00001;
            currentPower = Range.clip(currentPower, 0.1, 1.0);
        } else {
            currentPower=power;
        }
        double xValue = Math.sin(toRadians(targetHeading)) * currentPower;
        double yValue = Math.cos(toRadians(targetHeading)) * currentPower;
        // calculates required speed to adjust to gyStartAngle
        double angleError = (startAngle - currentAngle) / 25;
        // Setting range of adjustments
        angleError = Range.clip(angleError, -1, 1);

        if (distanceMovedInCM >= distance)
        {
            // Stops when at the specified distance
            stop();
            moving = false;
            return true;
        }
        else
        {
            // Continues if not at the specified distance
            telemetry.addData("y value", yValue);
            telemetry.addData("x value", xValue);
            telemetry.addData("angle error", angleError);

            joystickDrive(yValue, xValue, angleError, 1);
            return false;
        }
    }

    /**
     * Drive the robot in the heading provided using the internal imu.  It will rotate to the heading
     * and tank drive along that heading.
     * @param targetHeading  the heading the robot should drive
     * @param distance the distance the robot should drive before stopping
     * @param power the speed the robot should drive
     * @return boolean indicating true when the move is complete
     */
    public boolean gyroDrive(double targetHeading, double distance, double power)
    {

        double currentAngle = heading;

        // Set desired angle and initial distanceMoved
        if (!moving)
        {
            startAngle = currentAngle;
            initPosition = (long) leftFrontPosition;
            moving = true;
        }

        double distanceMoved;

        distanceMoved = abs(leftFrontPosition - initPosition);

        double distanceMovedInCM = distanceMoved / COUNTS_PER_CM;

        double distanceRemaining = Math.abs(distanceMovedInCM - distance);

        if (distance > 30) {
            if (distanceRemaining < 30) {
                power = (distanceRemaining / 30) * power;
                if (power < 0) {
                    power = Range.clip(power, -1.0, -0.1);
                } else {
                    power = Range.clip(power, 0.1, 1.0);
                }

            }
        }

        telemetry.addData("distanceMoved", distanceMoved);

        // calculates required speed to adjust to gyStartAngle
        double angleError = (targetHeading - currentAngle) / 20;
        // Setting range of adjustments
        angleError = Range.clip(angleError, -1, 1);

        if (distanceMovedInCM >= distance)
        {
            // Stops when at the specified distance
            stop();
            moving = false;
            return true;
        }
        else
        {
            // Continues if not at the specified distance
            joystickDrive(power, 0, angleError, 1);
            telemetry.addData("power: ", power);
            telemetry.addData("Angle error", angleError);
            return false;
        }
    }

    /**
     * Turns the robot an amount of degrees using the imu
     *
     * @param degreesToTurn - Angle the robot will turn
     * @param power   - Speed the robot will turn
     *
     * @return boolean indicating true when the move is complete
     */
    public boolean turn(double degreesToTurn, double power)
    {
        // Updates current angle
        double currentAngle = heading;

        // Sets initial angle
        if (!moving)
        {
            startAngle = heading;
            moving = true;
        }

        power = abs(power);
        if (degreesToTurn < 0)
        {
            // Make power a negative number if the degreesTo to turn is negative
            power = -power;
        }

        if (Math.abs(currentAngle - startAngle) >= abs(degreesToTurn))
        {
            // Stops turning when at the specified angle
            stop();
            moving = false;
            return true;
        }
        else
        {
            // Continues to turn if not at the specified angle
            joystickDrive(0, 0, power, 1);
            return false;
        }
    }

    /**
     * Turns the robot to a face a desired heading
     * @param targetHeading
     * @param power
     * @return boolean indicating true when the move is complete
     */
    public boolean turnTo(double targetHeading, double power)
    {
        // Updates current angle
        double currentAngle = heading;

        if (!moving)
        {
            startAngle = currentAngle;
            moving = true;
        }

        double angleError = targetHeading - currentAngle;
        double angleErrorMagnitude = Math.abs(angleError);

        if (angleError < 0.0)
        {
            power *= -1.0;
        }

        // If the difference between the current angle and the target angle is small (<10), scale
        // the power proportionally to how far you have left to go.  But... don't let the power
        // get too small because the robot won't have enough power to complete the turn if the
        // power gets too small.
        if ( angleErrorMagnitude < 10)
        {
            power = power * angleErrorMagnitude / 50.0;

            if (power > 0)
            {
                power = Range.clip(power, 0.1, 1);
            }
            else
            {
                power = Range.clip(power, -1, -0.1);
            }
        }

        if ( angleErrorMagnitude <= 1.5)
        {
            // Stops turning when at the specified angle (or really close)
            stop();
            moving = false;
            return true;
        }
        else
        {
            // Continues to turn if not at the specified angle
            joystickDrive(0, 0, power, 1);
            telemetry.addData("power", power);
            telemetry.addData("angle error", angleError);
            telemetry.addData("target heading", targetHeading);
            return false;
        }
    }


    public boolean strafe(ThunderBot_Claw_2024.Direction dir, double distance, double power)
    {
        if ( dir == ThunderBot_Claw_2024.Direction.LEFT)
        {
            return drive(-90, distance,  power);
        }
        else
        {
            return drive( 90, distance, power);
        }
    }

    public boolean alignToSpecimen( double power )
    {
        double Kp = 0.5;
        double targetX = limelight.getTargetX(false);
        if (targetX > 1)  // FIX THIS! - what is a proper termination condition?
        {
            double powertoStrafe = Kp*limelight.getTargetX(false);
            led.setToColor("purple");
            joystickDrive(0, powertoStrafe, 0, power);
            return false;
        }
        else
        {
            stop();
            led.setToColor("green");
            return true;
        }
    }

    public double specimenOffsetX(boolean isBlue){
        double Kp = 10.0;
        double targetX = limelight.getTargetX(isBlue) * Kp;

        if(targetX > 1)
        {
            targetX = 1;
        }
        else if(targetX < -1)
        {
            targetX = -1;
        }
        else if (targetX == 0.0)
        {
            return 0.0;
        }
        if(Math.abs(targetX / Kp) < 0.0254)
        {
            led.setToColor("green");
            return 0.0;
        }
        else
        {
            led.setToColor("purple");
            return targetX;
        }
    }
    public double specimenOffsetXSmooth(boolean isBlue){
        double Kp = 1.5;
        double targetX = limelight.getTargetXSmooth(isBlue) * Kp;

        if(targetX > 1)
        {
            targetX = 1;
        }
        else if(targetX < -1)
        {
            targetX = -1;
        }
        else if (targetX == 0.0)
        {
            return 0.0;
        }
        if(Math.abs(targetX / Kp) < 0.0254)
        {
            led.setToColor("green");
            return 0.0;
        }
        else
        {
            led.setToColor("purple");
            return targetX;
        }
    }
    public double specimenAngle(boolean isBlue){
        double Kp = 0.1;
        double targetH = limelight.updateHeading(isBlue);

        if(targetH > 10)
        {
            targetH = 10;
        }
        else if(targetH < -10)
        {
            targetH = -10;
        }
        else if(targetH == 0)
        {
            return 0;
        }

        if(Math.abs(targetH) < 1)
        {
            return 0;
        }
        else
        {
            return targetH * Kp;
        }
    }
    public double specimenAngleSmooth(boolean isBlue){
        double Kp = 1.0;
        double targetH = limelight.updateHeadingSmooth(isBlue);

        return targetH * Kp;
    }
    public double moveToHeading(double heading, double target){
        double Kp = 5.0;
        double targetH = Math.sin((heading-target)/Math.PI);

        return Range.clip(targetH*Kp, -1, 1);
    }

    public Action alignToSpecimenAction ( double power){
        return new Action() {
            private double pow = power;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return alignToSpecimen(pow);
            }
        };
    }
    public double colorOffsetX(){
        detectedBlock = pixy.getBlock();
        if(detectedBlock.isValid()) {
            return Math.sin((double) (detectedBlock.centerX - 75) /150);
        }else{
            return 0;
        }
    }
    public double colorOffsetY(){
        detectedBlock = pixy.getBlock();
        if(detectedBlock.isValid()) {
            return Math.sin((double) (detectedBlock.centerY - 163) /150);
        }else{
            return 0;
        }
    }
    public double colorOffsetAngle(){
        return Range.scale(detectedBlock.angle, -Math.PI,Math.PI,0,1);
    }

    public void updatePixy(){
        detectedBlock = pixy.getBlock();

    }

    public Action alignToColorAction(double Timeout){
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started){
                    started=true;
                    timer.reset();
                }
                if(timer.seconds() > Timeout){
                    return false;
                }else{
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(drive.pose.position.x+colorOffsetX(), drive.pose.position.y+colorOffsetY()))
                            .build();
                }
                return true;
            }
        };
    }


    private void update()
    {
        // Bulk data read.  MUST DO THIS EACH TIME THROUGH loop()
        for (LynxModule module : allHubs)
        {
            module.clearBulkCache();
        }
        drive.updatePoseEstimate();

        // Need to move these to not require motor encoders since the Into the Deep season won't use them
        heading = drive.pose.heading.toDouble();
        leftFrontPosition = drive.leftFront.getCurrentPosition();
        leftRearPosition = drive.leftBack.getCurrentPosition();
        rightFrontPosition = drive.rightFront.getCurrentPosition();
        rightRearPosition = drive.rightBack.getCurrentPosition();

        telemetry.addData("Motor Position", drive.pose);
        telemetry.addData("Heading: ", drive.pose.heading);

    }


    public void stop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

}

