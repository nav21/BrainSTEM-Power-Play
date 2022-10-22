package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.components.Claw;
import org.firstinspires.ftc.teamcode.components.Lift;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "auto")
public class BluePark extends LinearOpMode {
    public static double DISTANCE = 24; // inches

    @Override
    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this);
        robot.initAuto();
        /*
        Pose2d startPose = new Pose2d(-12, 65.5, Math.toRadians(0));
        Trajectory trajectory1 = robot.drive.trajectoryBuilder(startPose)
                .back(DISTANCE*1.1)
                .build();
        */

        waitForStart();

        if (isStopRequested()) return;
        /*
        robot.depositor.setGoal(Depositor.Goal.OPEN_LOOP);
        robot.drive.followTrajectory(trajectory1);
        */


        Pose2d poseEstimate = robot.drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive());
    }
}
