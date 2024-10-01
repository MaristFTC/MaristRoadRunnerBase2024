package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class SampleRoadrunnerAuto extends LinearOpMode {

    MaristBaseRobot2024_Quad_RoadRunner robot = new MaristBaseRobot2024_Quad_RoadRunner();
    Pose2d startPose = new Pose2d(0,0,0);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //Simple Trajectory Actionwhich should move from position (0,0) with a heading of 0 (facing to the right) to position (30,30) with a heading of 90 (facing forward) in a continuous smooth motion
        Action simpleMoveForwardAction = robot.drive.actionBuilder(startPose).strafeToLinearHeading(new Vector2d(30,30), Math.toRadians(90)).build();
        waitForStart();

        //Run the Trajectory
        Actions.runBlocking(simpleMoveForwardAction);



    }
}
