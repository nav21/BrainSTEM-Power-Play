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


        Pose2d startPose = new Pose2d(-36, -66, Math.toRadians(-90));
        robot.drive.setPoseEstimate(startPose);
        Trajectory traj = robot.drive.trajectoryBuilder(startPose,true)
                .lineToSplineHeading(new Pose2d(-36, -26, Math.toRadians(-180)), BMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.5, () -> robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID))
//                .addTemporalMarker(1.1, () -> robot.lift.setGoal(Lift.Goal.UP))
//                .addTemporalMarker(2, () -> robot.claw.setCurrentGoal(Claw.Goal.FLIP))
                .build();
        Trajectory traj2 = robot.drive.trajectoryBuilder(traj.end())
                .lineTo(new Vector2d(-30, -26))
//                .addTemporalMarker(1.5, () -> robot.claw.setCurrentGoal(Claw.Goal.RELEASE))
//                .addTemporalMarker(2, () -> robot.claw.setCurrentGoal(Claw.Goal.RETURN_MID))
//                .addTemporalMarker(2.5, () -> robot.lift.setGoal(Lift.Goal.DOWN))
                .build();
        Trajectory traj23 = robot.drive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-36,-26))
///                .addTemporalMarker(1, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
                .build();
       Trajectory traj3 = robot.drive.trajectoryBuilder(traj23.end())
                .lineToConstantHeading(new Vector2d(-42, -14))
///                .addTemporalMarker(1, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
                .build();
        Trajectory traj4 = robot.drive.trajectoryBuilder(traj3.end())
                .lineToConstantHeading(new Vector2d(-65, -14))
////                .addTemporalMarker(1.5, () -> robot.claw.setCurrentGoal(Claw.Goal.OLLECT_MID))
                .build();
        Trajectory traj5 = robot.drive.trajectoryBuilder(traj4.end(), true)
                .splineTo(new Vector2d(-27, -21), Math.toRadians(-45), BMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // TODO ENABLE TRAJ 6 IF USING LINE TO SPLINE
                //  .lineToSplineHeading(new Pose2d(-36, -12, Math.toRadians(-225)))
//                .addTemporalMarker(0.01, () -> robot.lift.setGoal(Lift.Goal.UP))
//                .addTemporalMarker(1, () -> robot.claw.setCurrentGoal(Claw.Goal.FLIP))
//                .addTemporalMarker(1.7, () -> robot.claw.setCurrentGoal(Claw.Goal.RELEASE))
//                .addTemporalMarker(2.1, () -> robot.claw.setCurrentGoal(Claw.Goal.RETURN_MID))
                .build();
//        Trajectory traj6 = robot.drive.trajectoryBuilder(traj5.end())
//                // TODO: USE THIS TRAJ IF USING LINE TO SPLINE IN TRAJ 5
//                .lineTo(new Vector2d(-30, -18))
//                // .addTemporalMarker(0.01, () -> robot.lift.setGoal(Lift.Goal.UP))
//                .build();
//        Trajectory traj7 = robot.drive.trajectoryBuilder(traj5.end())
//                // TODO: SEE IF YOU CAN REVERSE THE PATH BY JUST SPLINING BACK
//                .lineTo(new Vector2d(-36, -12))
////                 .addTemporalMarker(0.5, () -> robot.lift.setGoal(Lift.Goal.DOWN))
////                .addTemporalMarker(0.8, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
//                .build();
//        Trajectory traj8 = robot.drive.trajectoryBuilder(traj7.end())
//                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(-180)))
//                // .addTemporalMarker(1.5, () -> robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID))
//                .build();
//        robot.lift.setGoal(Lift.Goal.OPEN_LOOP);
//        robot.claw.setCurrentGoal(Claw.Goal.OPEN_LOOP);
//        //   robot.lift.setMode(Lift.Mode.MED);
        Trajectory traj9 = robot.drive.trajectoryBuilder(traj5.end())
                .splineTo(new Vector2d(-64, -12), Math.toRadians(-180) , BMecanumDrive.getVelocityConstraint(26, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // TODO ENABLE TRAJ 6 IF USING LINE TO SPLINE
                //  .lineToSplineHeading(new Pose2d(-36, -12, Math.toRadians(-225)))
//                .addTemporalMarker(0.01, () -> robot.lift.setGoal(Lift.Goal.UP))
//                .addTemporalMarker(1, () -> robot.claw.setCurrentGoal(Claw.Goal.FLIP))
//                .addTemporalMarker(1.7, () -> robot.claw.setCurrentGoal(Claw.Goal.RELEASE))
//                .addTemporalMarker(2.1, () -> robot.claw.setCurrentGoal(Claw.Goal.RETURN_MID))
                .build();
        Trajectory traj10 = robot.drive.trajectoryBuilder(traj9.end(), true)
                .splineTo(new Vector2d(-27, -21), Math.toRadians(-45), BMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // TODO ENABLE TRAJ 6 IF USING LINE TO SPLINE
                //  .lineToSplineHeading(new Pose2d(-36, -12, Math.toRadians(-225)))
//                .addTemporalMarker(0.01, () -> robot.lift.setGoal(Lift.Goal.UP))
//                .addTemporalMarker(1, () -> robot.claw.setCurrentGoal(Claw.Goal.FLIP))
//                .addTemporalMarker(1.7, () -> robot.claw.setCurrentGoal(Claw.Goal.RELEASE))
//                .addTemporalMarker(2.1, () -> robot.claw.setCurrentGoal(Claw.Goal.RETURN_MID))
                .build();
        robot.drive.followTrajectory(traj);
        sleep(200);
        robot.drive.followTrajectory(traj2);
        sleep(2000);
        robot.drive.followTrajectory(traj23);
        sleep(200);
        robot.drive.followTrajectory(traj3);
        sleep(200);
        robot.drive.followTrajectory(traj4);
        sleep(2000);
        robot.drive.followTrajectory(traj5);
        sleep(2000);
        robot.drive.followTrajectory(traj9);
        sleep(2000);
        robot.drive.followTrajectory(traj10);

//        robot.drive.followTrajectory(traj7);
//        sleep(200);
//        robot.drive.followTrajectory(traj8);

    }
}

