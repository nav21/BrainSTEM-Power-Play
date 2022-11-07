package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.enums.ClawPosition;
import org.firstinspires.ftc.teamcode.autonomous.enums.LiftPosition;
import org.firstinspires.ftc.teamcode.components.AutoBrainSTEMRobot;
import org.firstinspires.ftc.teamcode.components.Claw;
import org.firstinspires.ftc.teamcode.components.Lift;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "auto")
public class redAutoWare extends LinearOpMode {
    public static double DISTANCE = 24; // inches

    @Override
    public void runOpMode() throws InterruptedException {
        AutoBrainSTEMRobot robot = new AutoBrainSTEMRobot(this);

        robot.initAuto();
        /*
        Pose2d startPose = new Pose2d(-12, -63.5, Math.toRadians(-62));

        Trajectory trajectory2 = robot.drive.trajectoryBuilder(startPose.plus(new Pose2d(0, 0, Math.toRadians(28))))
                .back(DISTANCE*1.3)
                .addTemporalMarker(0, () -> {
                    robot.depositor.runLiftToPosition(Depositor.Mode.HIGH);
                })
                .addTemporalMarker(1, () -> {
                    robot.depositor.setFlipDepositorServoPosition(DepositorPosition.OUT);
                })
                .addTemporalMarker(2, () -> {
                    robot.depositor.setExtendServoPosition(DepositorPosition.OUT);
                })
                .addTemporalMarker(4, () -> {
                    robot.depositor.setDepositorGateServoPosition(DepositorPosition.OUT);
                })
                .build();
        Trajectory trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end())
                .forward(DISTANCE*1.35)
                .addTemporalMarker(1, () -> {
                    robot.depositor.setFlipDepositorServoPosition(DepositorPosition.IN);
                    robot.depositor.setExtendServoPosition(DepositorPosition.IN);
                    robot.depositor.setDepositorGateServoPosition(DepositorPosition.IN);
                })
                .addTemporalMarker(4, () -> {
                    robot.depositor.runLiftToPosition(Depositor.Mode.LOW);
                })
                .build();
        Trajectory trajectory4 = robot.drive.trajectoryBuilder(trajectory3.end().plus(new Pose2d(0, 0, Math.toRadians(46))))
                .forward(DISTANCE*1.1)
                .build();
        Trajectory trajectory5 = robot.drive.trajectoryBuilder(trajectory4.end().plus(new Pose2d(0,0, Math.toRadians(10))))
                .forward(DISTANCE)
                .addTemporalMarker(0.5, () -> {
                   robot.start();
                   robot.collector.setCurrentGoal(Collector.Goal.COLLECT);
                })
                .addTemporalMarker(4, () -> {
                   robot.stop();
                })
                .build();
        Trajectory trajectory6 = robot.drive.trajectoryBuilder(trajectory5.end().plus(new Pose2d(0,0, Math.toRadians(-2))))
                .back(DISTANCE*2)
                .addTemporalMarker(0.5, () -> {
                    robot.collector.setGateServoPosition(CollectorPosition.OUT);
                    robot.collector.setCollectorPower(0);
                })
                .addTemporalMarker(1, () -> {
                    robot.collector.setCollectorPower(1);
                })
                .addTemporalMarker(2.5, () -> {
                    robot.collector.setCollectorPower(0);
                })
                .build();
        Trajectory trajectory7 = robot.drive.trajectoryBuilder(trajectory6.end().plus(new Pose2d(0, 0, Math.toRadians(-70))))
                .back(DISTANCE*1.3)
                .addTemporalMarker(0, () -> {
                    robot.depositor.runLiftToPosition(Depositor.Mode.HIGH);
                })
                .addTemporalMarker(1, () -> {
                    robot.depositor.setFlipDepositorServoPosition(DepositorPosition.OUT);
                })
                .addTemporalMarker(2, () -> {
                    robot.depositor.setExtendServoPosition(DepositorPosition.OUT);
                })
                .addTemporalMarker(4, () -> {
                    robot.depositor.setDepositorGateServoPosition(DepositorPosition.OUT);
                })
                .build();
        Trajectory trajectory8 = robot.drive.trajectoryBuilder(trajectory7.end())
                .forward(DISTANCE*1.2)
                .addTemporalMarker(1, () -> {
                    robot.depositor.setFlipDepositorServoPosition(DepositorPosition.IN);
                    robot.depositor.setExtendServoPosition(DepositorPosition.IN);
                    robot.depositor.setDepositorGateServoPosition(DepositorPosition.IN);
                })
                .addTemporalMarker(4, () -> {
                    robot.depositor.runLiftToPosition(Depositor.Mode.LOW);
                })
                .build();
        Trajectory trajectory9 = robot.drive.trajectoryBuilder(trajectory8.end().plus(new Pose2d(0, 0, Math.toRadians(51))))
                .forward(DISTANCE*1.35)
                .build();
        Trajectory trajectory10 = robot.drive.trajectoryBuilder(trajectory9.end().plus(new Pose2d(0,0, Math.toRadians(11))))
                .forward(DISTANCE)
                .addTemporalMarker(0.5, () -> {
                    robot.start();
                    robot.collector.setCurrentGoal(Collector.Goal.COLLECT);
                })
                .addTemporalMarker(4, () -> {
                    robot.stop();
                })
                .build();
        */
        waitForStart();

        if (isStopRequested()) return;
        /*
        robot.depositor.setGoal(Depositor.Goal.OPEN_LOOP);
        robot.drive.turn(Math.toRadians(28));
        robot.drive.followTrajectory(trajectory2);
        sleep(2000);
        robot.drive.followTrajectory(trajectory3);
        robot.drive.turn(Math.toRadians(46));
        robot.drive.followTrajectory(trajectory4);
        robot.drive.turn(Math.toRadians(10));
        robot.drive.followTrajectory(trajectory5);
        robot.drive.turn(Math.toRadians(-2));
        robot.drive.followTrajectory(trajectory6);
        robot.drive.turn(Math.toRadians(-70));
        robot.drive.followTrajectory(trajectory7);
        sleep(2000);
        robot.drive.followTrajectory(trajectory8);
        robot.drive.turn(Math.toRadians(51));
        robot.drive.followTrajectory(trajectory9);
        robot.drive.turn(Math.toRadians(11));
        robot.drive.followTrajectory(trajectory10);
        */


        Pose2d poseEstimate = robot.drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive());
    }
}
