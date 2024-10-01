package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


/**
 * Created by michaudc on 10/8/2017.
 * Updated by michaudc on 29 July 2021
 * Additional code by Allesio Toniolo July 2021
 * Based on HardwarePushbot Code from the FTCRobotController resources
 * 
 * Revision for 2022 Season V1 on 19 Aug 22 by michaudc
 *
 * Revision for 2023 Season V1 on 24 July 22 by michaudc
 * Added Java Karel style method calls for move, turnLeft, turnRight, turn
 * Overloaded Methods with (distance, speed) and (distance) formats
 * 
 * Revision for 2024-2025 Season V1 05 Jun 24 by Cole S, Russell M, and michaudc
 * Added methods for field centric drive in Base robot
 * Revision for 2024 Updated getBrightness() to return value from HSV
 * 
 * This class models the physical structure of the robot with instances
 * of motors, servos, and sensors.
 *
 * The following are name assignments to be configured
 * on the RC Phone in the App.
 *
 * Motor channel: leftFront:        "leftfront"
 * Motor channel: rightFront:       "rightfront"
 * Motor channel: leftRear:          "leftrear"
 * Motor channel: rightRear:         "rightrear"
 * Motor channel: leftArm:          "leftarm"
 * Motor channel: rightArm:         "rightarm"
 * Servo Channel: leftHand:         "lefthand"
 * Servo Channel: rightHand:        "righthand"
 * Touch Sensor:  touch             "touch" ** Digital 1 in Config
 * Color Sensor:  colorSensor       "colorSensor"
 *
 */

public class MaristBaseRobot2024_Quad_RoadRunner {
    /* Public Motors and Servos */

    public DcMotor rightArm    = null;
    public DcMotor leftArm     = null;
    public Servo rightHand    = null;
    public Servo leftHand   = null;

    public   IMU imu;
    public double offset = 0;//in degrees



    /* Public Sensors */
    public DigitalChannel touch = null;
    public NormalizedColorSensor colorSensor = null;

    // Constants for Arm and Servo Operation
    public static final double MID_SERVO        =  0.5;
    public static final double ARM_UP_POWER     =  2.00;
    public static final double ARM_DOWN_POWER   = -2.00;

    private double     COUNTS_PER_MOTOR_REV          = 1440 ;    // eg: TETRIX Motor Encoder
    private double COUNTS_PER_DEGREE                 = COUNTS_PER_MOTOR_REV / 360;
    private double     DRIVE_SPEED                   = 0.6;
    private double     TURN_SPEED                    = 0.5;

    // Local OpMode members
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    MecanumDrive drive;
    Pose2d startPose = new Pose2d(0,0,0);

    // Constructor - leave this blank for now
    public MaristBaseRobot2024_Quad_RoadRunner() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        drive = new MecanumDrive(hwMap, startPose);
        // Define and Initialize Motors.  Assign Names that match the setup on the RC Phone

        leftArm      = hwMap.dcMotor.get("leftarm");
        rightArm     = hwMap.dcMotor.get("rightarm");

        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.FORWARD);





        // Define and initialize ALL installed servos.
        leftHand = hwMap.servo.get("lefthand");
        rightHand = hwMap.servo.get("righthand");
        leftHand.setPosition(MID_SERVO);
        rightHand.setPosition(MID_SERVO);

        // Define and Initialize Sensors
        touch = hwMap.get(DigitalChannel.class, "touch");
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        
        // IMU Sensor for Field Centric Driving
        imu = hwMap.get(IMU.class, "imu");

        // Define for Orientation of REV Control Hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // Initialize the IMU
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        
        touch.setMode(DigitalChannel.Mode.INPUT);

    }


    /**
     * Drive Method Using the Roadrunner Mecanum Drive class
     * Takes in parameters:
     * xPow (power to move in the x direction; side to side)
     * yPow (power to move in the y direciton; forward in backward)
     * rotPow (power to move rotational; how fast to turn the robot)
     *
     */

    public void driveRobot(double xPow, double yPow, double rotPow){
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(xPow,yPow),rotPow));
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    // Additional Functions to control Servos and motors



    public void leftArmMotorDeg(double speed,
                            double deg,
                            double timeoutS) {
        int target;

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            target = leftArm.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            leftArm.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            leftArm.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    leftArm.isBusy()) {
                // Wait for Sequence to complete
            }

            // Stop all motion: Comment out if you want Motor to hold position
            //leftArm.setPower(0);

            // Turn off RUN_TO_POSITION: Comment out if you want Motor to hold position
            //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    public void rightArmMotorDeg(double speed,
                             double deg,
                             double timeoutS) {
        int target;

        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            target = rightArm.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            rightArm.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            rightArm.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    rightArm.isBusy()) {
                // Wait for Sequence to complete
            }

            // Stop all motion: Comment out if you want Motor to hold position
            //rightArm.setPower(0);

            // Turn off RUN_TO_POSITION: Comment out if you want Motor to hold position
            //rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }


    // Functions for Color Sensor - July 2021
    
    public float [] getColorValues() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float [] output = new float [3];
        output[0] = colors.red;
        output[1] = colors.blue;
        output[2] = colors.green;
        return output;
    }
    
    public double getRed() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return colors.red;
    }
    
    public double getGreen() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return colors.green;
    }
    
    public double getBlue() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return colors.blue;
    }
    
    public float getIntensity() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        return hsvValues[2];
    }

    // Added by Mr. Michaud 19 Sep 22
    public float getHue() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        return hsvValues[0];
    }
    //added by Cole Saunders 22 Oct 23
    public double getOrientation(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES)-offset;
    }
    public double resetImu(){
        offset=  imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
        return offset;
    }

    

}


