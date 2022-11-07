package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.util.NanoClock;


import org.firstinspires.ftc.teamcode.autonomous.enums.ClawPosition;
import org.firstinspires.ftc.teamcode.autonomous.enums.LiftPosition;
import org.firstinspires.ftc.teamcode.autonomous.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.vision.VisionLibrary;
import org.firstinspires.ftc.teamcode.autonomous.vision.SignalSleevePosition;
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

    public void runMain(AutoBrainSTEMRobot robot, SignalSleevePosition signalSleevePosition) {
        /*
        // Example of 'Async' use to allow HW updating for things like slower servos and SWPIDs
        // It's possible that your 'auto' class already handles this with threads, but I'm not sure.

        // Tell Roadrunner what to do
        robot.drive.followTrajectoryAsync(trajectory1);
        // This will update drive and components until the drive motion is completed.
        CheckWait(true, true, 0, 0);

        robot.drive.turnAsync(Math.toRadians(-44));
        CheckWait(true, true, 0, 0);

        robot.drive.followTrajectoryAsync(trajectory2);
        // This is a better 'sleep', it will keep updating our components for 5500ms even if the drive motion completes
        CheckWait(true, true, 5500, 5500);
        */


        Pose2d startPose = new Pose2d(-36, -64.38, Math.toRadians(-90));
        robot.drive.setPoseEstimate(startPose);
        Trajectory traj = robot.drive.trajectoryBuilder(startPose,true)
                .lineToSplineHeading(new Pose2d(-36, -30, Math.toRadians(-181)))
                .build();
        Trajectory traj2 = robot.drive.trajectoryBuilder(traj.end())
                .lineTo(new Vector2d(-30, -30))
                .build();
        Trajectory traj3 = robot.drive.trajectoryBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(-34, -8))
                .build();
        Trajectory traj4 = robot.drive.trajectoryBuilder(traj3.end())
                .lineToConstantHeading(new Vector2d(-62, -8))
                .build();
        robot.drive.followTrajectory(traj);
        sleep(2000);
        // robot.drive.followTrajectory(traj2);
        robot.drive.followTrajectory(traj3);
        sleep(200);
        robot.drive.followTrajectory(traj4);
    }
}
