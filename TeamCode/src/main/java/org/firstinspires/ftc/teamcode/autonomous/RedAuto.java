package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.enums.ClawPosition;
import org.firstinspires.ftc.teamcode.autonomous.enums.LiftPosition;
import org.firstinspires.ftc.teamcode.autonomous.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.vision.FFVisionLibrary;
import org.firstinspires.ftc.teamcode.autonomous.vision.TeamMarkerDetector;
import org.firstinspires.ftc.teamcode.autonomous.vision.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.buttons.StickyButton;
import org.firstinspires.ftc.teamcode.components.AutoBrainSTEMRobot;
import org.firstinspires.ftc.teamcode.components.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.components.Claw;
import org.firstinspires.ftc.teamcode.components.Lift;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "auto")
public class RedAuto extends BaseAuto {
    public static double DISTANCE = 24; // inches

    public void runMain(AutoBrainSTEMRobot robot, TeamMarkerPosition teamMarkerPosition) {
        Pose2d startPose = new Pose2d(-36, -63.5, Math.toRadians(-90));
        /*
        Trajectory trajectory1 = robot.drive.trajectoryBuilder(startPose)
                .back(DISTANCE * 0.9)
                .addTemporalMarker(1, () -> {
                    robot.depositor.runLiftToPosition(Depositor.Mode.HIGH);
                })
                .build();
        Trajectory trajectory2 = robot.drive.trajectoryBuilder(trajectory1.end().plus(new Pose2d(0, 0, Math.toRadians(-44))))
                .back(DISTANCE * 0.58)
                .addTemporalMarker(0.5, () -> {
                    robot.depositor.setFlipDepositorServoPosition(DepositorPosition.OUT);
                })
                .addTemporalMarker(1, () -> {
                    robot.depositor.setExtendServoPosition(DepositorPosition.OUT);
                })
                .addTemporalMarker(1.5, () -> {
                    robot.depositor.setDepositorGateServoPosition(DepositorPosition.OUT);
                })
                .addTemporalMarker(4.5, () -> {
                    robot.depositor.setExtendServoPosition(DepositorPosition.IN);
                    robot.depositor.setDepositorGateServoPosition(DepositorPosition.IN);
                    robot.depositor.setFlipDepositorServoPosition(DepositorPosition.IN);
                })
                .addTemporalMarker(7, () -> {
                    robot.depositor.runLiftToPosition(Depositor.Mode.LOW);
                })
                .build();
        Trajectory trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end().plus(new Pose2d(0, 0, Math.toRadians(-12))))
                .forward(DISTANCE * 2)
                .build();
        Trajectory trajectory4 = robot.drive.trajectoryBuilder(trajectory2.end().plus(new Pose2d(0, 0, Math.toRadians(52))))
                .back(DISTANCE * 1.05)
                .build();
        Trajectory trajectory5 = robot.drive.trajectoryBuilder(trajectory4.end().plus(new Pose2d(0, 0, Math.toRadians(-70))))
                .forward(DISTANCE)
                .build();

        robot.depositor.setGoal(Depositor.Goal.OPEN_LOOP);
        robot.drive.followTrajectory(trajectory1);
        robot.drive.turn(Math.toRadians(-44));
        robot.drive.followTrajectory(trajectory2);
        sleep(5500);
        robot.drive.turn(Math.toRadians(-12));
        robot.drive.followTrajectory(trajectory3);
        robot.collector.setCollectorPower(-0.4);
        sleep(4000);
        robot.collector.setCollectorPower(0);
        sleep(1000);
        robot.drive.turn(Math.toRadians(52));
        robot.drive.followTrajectory(trajectory4);
    */
    }
}
