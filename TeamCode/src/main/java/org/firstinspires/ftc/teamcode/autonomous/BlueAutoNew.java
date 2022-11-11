package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.enums.ClawPosition;
import org.firstinspires.ftc.teamcode.autonomous.enums.LiftPosition;
import org.firstinspires.ftc.teamcode.components.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.components.Claw;
import org.firstinspires.ftc.teamcode.components.Lift;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Disabled
@Autonomous(group = "auto")
public class BlueAutoNew extends LinearOpMode {
    public static double DISTANCE = 24; // inches

    @Override
    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this);
        robot.initAuto();
        /*
        Pose2d startPose = new Pose2d(-36, 63.63, Math.toRadians(90));
        Trajectory trajectory1 = robot.drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-20, 30), Math.toRadians(45))
                .build();
        Trajectory trajectory2 = robot.drive.trajectoryBuilder(trajectory1.end().plus(new Pose2d(0, 0, Math.toRadians(36))))
                .back(DISTANCE*0.62)
                .build();
        Trajectory trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end())
                .forward(DISTANCE*1.46)
                .build();
        Trajectory trajectory4 = robot.drive.trajectoryBuilder(trajectory3.end().plus(new Pose2d(0, 0, Math.toRadians(31))))
                .forward(DISTANCE*0.4)
                .build();
        Trajectory trajectory5 = robot.drive.trajectoryBuilder(trajectory4.end().plus(new Pose2d(0,0, Math.toRadians(80))))
                .forward(DISTANCE*1.05)
                .build();
        */


        waitForStart();

        if (isStopRequested()) return;
        /*
        robot.depositor.setGoal(Depositor.Goal.OPEN_LOOP);
        robot.drive.followTrajectory(trajectory1);
        sleep(500);
        robot.depositor.setLiftPower(0.7);
        sleep(1000);
        robot.depositor.setLiftPower(0.01);
        sleep(1000);
        sleep(1000);
        robot.depositor.setLiftPower(-0.7);
        sleep(800);
        robot.depositor.setLiftPower(0);
        robot.drive.followTrajectory(trajectory3);
        robot.drive.turn(Math.toRadians(31));
        robot.drive.followTrajectory(trajectory4);
        robot.collector.setCollectorPower(0.36);
        sleep(4000);
        robot.collector.setCollectorPower(0);
        robot.drive.turn(Math.toRadians(80));
        robot.drive.followTrajectory(trajectory5);
        */

        Pose2d poseEstimate = robot.drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive());
    }
}
