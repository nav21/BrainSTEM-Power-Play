package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous(group = "auto")
public class RedPark extends LinearOpMode {
    public static double DISTANCE = 36; // inches

    @Override
    public void runOpMode() throws InterruptedException {
        AutoBrainSTEMRobot robot = new AutoBrainSTEMRobot(this);


        robot.initAuto();
        robot.start();
        /*
        Pose2d startPose = new Pose2d(-12, -63.5, Math.toRadians(-90));
        Trajectory trajectory1 = robot.drive.trajectoryBuilder(startPose)
                .back(DISTANCE*0.04)
                .build();
        Trajectory trajectory2 = robot.drive.trajectoryBuilder(trajectory1.end().plus(new Pose2d(0, 0, Math.toRadians(45))))
                .addTemporalMarker(1, () -> {
                    robot.depositor.runLiftToPosition(Depositor.Mode.HIGH);
                })
                .back(DISTANCE*0.8)
                .addDisplacementMarker(10, () -> {
                    robot.depositor.setGoal(Depositor.Goal.UP);
                    robot.stop();
                    robot.depositor.setGoal(Depositor.Goal.OPEN_LOOP);
                    robot.depositor.setExtendServoPosition(DepositorPosition.OUT);
                })
                .addDisplacementMarker(() -> {
                    robot.depositor.setDepositorGateServoPosition(DepositorPosition.OUT);
                })
                .build();
        Trajectory trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end())
                .forward(DISTANCE*0.5)
                .addTemporalMarker(3, () -> {
                    robot.depositor.setFlipDepositorServoPosition(DepositorPosition.IN);
                    robot.depositor.setExtendServoPosition(DepositorPosition.IN);
                   robot.depositor.setDepositorGateServoPosition(DepositorPosition.IN);
                })
                .addTemporalMarker(3, () -> {
                    robot.depositor.runLiftToPosition(Depositor.Mode.LOW);
                })
                .build();
        */
        waitForStart();

        if (isStopRequested()) return;
        /*
        robot.drive.followTrajectory(trajectory1);
         robot.drive.turn(Math.toRadians(45));
        robot.drive.followTrajectory(trajectory2);
        sleep(3000);

        robot.drive.followTrajectory(trajectory3);
        */
        Pose2d poseEstimate = robot.drive.getPoseEstimate();

        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
        robot.stop();
        while (!isStopRequested() && opModeIsActive());
    }
}
