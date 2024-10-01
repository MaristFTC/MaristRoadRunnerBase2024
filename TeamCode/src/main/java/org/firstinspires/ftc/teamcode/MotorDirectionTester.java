package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


@TeleOp

public class MotorDirectionTester extends LinearOpMode{
    OverflowEncoder rightFrontE;
    OverflowEncoder rightRearE;
    OverflowEncoder leftRearE;
    OverflowEncoder leftFrontE;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");

        leftFrontE = new OverflowEncoder(new RawEncoder(leftFront));
        rightFrontE = new OverflowEncoder(new RawEncoder(rightFront));
        leftRearE = new OverflowEncoder(new RawEncoder(leftRear));
        rightRearE = new OverflowEncoder(new RawEncoder(rightRear));

        //Reverse motors as need until they all spin forward when you press the buttons
        //Remember to mirror this change in whatever Base Robot class you use


        telemetry.addLine("Hold \nX -> Left Front\n Y -> rightFront\nB -> rightRear\nA -> leftRear");
        telemetry.update();
        waitForStart();




        while(!isStopRequested() && opModeIsActive()){

            telemetry.addLine("Hold \nX -> Left Front\n Y -> rightFront\nB -> rightRear\nA -> leftRear");

            if(gamepad1.y)rightFront.setPower(1);
            else rightFront.setPower(0);
            if(gamepad1.b)rightRear.setPower(1);
            else rightRear.setPower(0);
            if(gamepad1.a)leftRear.setPower(1);
            else leftRear.setPower(0);
            if(gamepad1.x)leftFront.setPower(1);
            else leftFront.setPower(0);
            telemetry.addLine("LeftFront Ticks " + leftFrontE.getPositionAndVelocity().position);
            telemetry.addLine("RightFront Ticks " + rightFrontE.getPositionAndVelocity().position);
            telemetry.addLine("RightRear Ticks " + rightRearE.getPositionAndVelocity().position);
            telemetry.addLine("LeftRear Ticks" + leftRearE.getPositionAndVelocity().position);

        }
    }


}
