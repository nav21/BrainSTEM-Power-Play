package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.components.BMecanumDrive;
import org.firstinspires.ftc.teamcode.components.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.components.Claw;
import org.firstinspires.ftc.teamcode.components.DriveConstants;
import org.firstinspires.ftc.teamcode.components.Lift;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Disabled
@Autonomous(group = "auto")
public class RedAutoVision extends BaseAuto {
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


        Pose2d startPose = new Pose2d(-36, -66, Math.toRadians(-90));
        robot.drive.setPoseEstimate(startPose);
        Trajectory goToMedGoalPosition = robot.drive.trajectoryBuilder(startPose,true)
                .lineToSplineHeading(new Pose2d(-36, -24, Math.toRadians(-180)), BMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addTemporalMarker(0.01, () -> robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID))
//                .addTemporalMarker(0.6, () -> robot.lift.setGoal(Lift.Goal.UP))
//                .addTemporalMarker(1, () -> robot.claw.setCurrentGoal(Claw.Goal.FLIP))
                .build();
        Trajectory depositPreloadMedGoal = robot.drive.trajectoryBuilder(goToMedGoalPosition.end())
                .lineTo(new Vector2d(-30, -24))
//                .addTemporalMarker(0.25, () -> robot.claw.setCurrentGoal(Claw.Goal.RELEASE))
//                .addTemporalMarker(0.5, () -> robot.claw.setCurrentGoal(Claw.Goal.RETURN_MID))
//                .addTemporalMarker(1, () -> robot.lift.setGoal(Lift.Goal.DOWN))
                .build();
        Trajectory goToFirstConeAndGetReadyForPark = robot.drive.trajectoryBuilder(depositPreloadMedGoal.end())
//                .lineToConstantHeading(new Vector2d(-36, -24))
//                .lineToConstantHeading(new Vector2d(-42, -12))
               // .lineToConstantHeading(new Vector2d(-64, -12))
//                .addTemporalMarker(0.0, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
//                .addTemporalMarker(0.5, () -> robot.claw.setCurrentGoal(Claw.Goal.RETURN_MID))
//                .addTemporalMarker(1, () -> robot.lift.setGoal(Lift.Goal.DOWN))
                .build();
        robot.lift.setGoal(Lift.Goal.OPEN_LOOP);
        robot.claw.setCurrentGoal(Claw.Goal.OPEN_LOOP);
        robot.drive.followTrajectory(goToMedGoalPosition);
        sleep(200);
        robot.drive.followTrajectory(depositPreloadMedGoal);
        sleep(200);


    }
}