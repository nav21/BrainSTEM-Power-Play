package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.vision.SignalSleevePosition;
import org.firstinspires.ftc.teamcode.components.AutoBrainSTEMRobot;
import org.firstinspires.ftc.teamcode.components.BMecanumDrive;
import org.firstinspires.ftc.teamcode.components.Claw;
import org.firstinspires.ftc.teamcode.components.DriveConstants;
import org.firstinspires.ftc.teamcode.components.Lift;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Disabled
@Autonomous(group = "auto")
public class visionAutoRed2Cone extends BaseAuto {
    public static double DISTANCE = 24; // inches

    @Override
    public void buildPaths(AutoBrainSTEMRobot robot) {

    }

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


        Pose2d startPose = new Pose2d(-36, -64.6, Math.toRadians(-90));
        robot.drive.setPoseEstimate(startPose);
        Trajectory goToMedGoalPosition = robot.drive.trajectoryBuilder(startPose,true)
                .lineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(-180)), BMecanumDrive.getVelocityConstraint(26, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToConstantHeading(new Vector2d(-36, -25), BMecanumDrive.getVelocityConstraint(26, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID))
                .addTemporalMarker(0.7, () -> robot.lift.setGoal(Lift.Goal.UP))
                .addTemporalMarker(2, () -> robot.claw.setCurrentGoal(Claw.Goal.FLIP))
                .build();
        Trajectory depositPreloadMedGoal = robot.drive.trajectoryBuilder(goToMedGoalPosition.end())
                .lineTo(new Vector2d(-32, 25), BMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory goToFirstCone1 = robot.drive.trajectoryBuilder(depositPreloadMedGoal.end())
                .lineToConstantHeading(new Vector2d(-37, -25), BMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.5, () -> robot.lift.setMode(Lift.Mode.CONE_5))
                .build();
        Trajectory goToFirstCone2 = robot.drive.trajectoryBuilder(goToFirstCone1.end())
                .lineToConstantHeading(new Vector2d(-40, -14), BMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory goToFirstCone3 = robot.drive.trajectoryBuilder(goToFirstCone2.end())
                .lineToConstantHeading(new Vector2d(-64, -14), BMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
                .build();
        Trajectory strafeForPark = robot.drive.trajectoryBuilder(goToFirstCone1.end()) // TODO change this end trajectoy
                .lineToConstantHeading(new Vector2d(-37, -36), BMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory park1 = robot.drive.trajectoryBuilder(strafeForPark.end())
                .lineTo(new Vector2d(-60, 36), BMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.lift.setGoal(Lift.Goal.DOWN))
                .addTemporalMarker(0.4, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
                .build();
        Trajectory park3 = robot.drive.trajectoryBuilder(strafeForPark.end())
                .lineTo(new Vector2d(-14, 36), BMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.lift.setGoal(Lift.Goal.DOWN))
                .addTemporalMarker(0.4, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
                .build();



        robot.lift.setGoal(Lift.Goal.OPEN_LOOP);
        robot.claw.setCurrentGoal(Claw.Goal.OPEN_LOOP);
        robot.lift.setMode(Lift.Mode.MED);
        robot.drive.followTrajectory(goToMedGoalPosition);
        sleep(200);
        robot.drive.followTrajectory(depositPreloadMedGoal);
        robot.claw.setCurrentGoal(Claw.Goal.RELEASE);
        sleep(350);
        robot.claw.setCurrentGoal(Claw.Goal.RETURN_MID);
        sleep(400);
        robot.drive.followTrajectory(goToFirstCone1);
        sleep(750);
        robot.drive.followTrajectory(goToFirstCone2);
        sleep(750);
        robot.drive.followTrajectory(goToFirstCone3);
        sleep(750);
        robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID);
        robot.drive.followTrajectory(strafeForPark);
        sleep(1000);
        if (signalSleevePosition == SignalSleevePosition.ONE) {
            robot.drive.followTrajectory(park1);
        } else if (signalSleevePosition == SignalSleevePosition.THREE){
            robot.drive.followTrajectory(park3);
        }
        else {
            robot.lift.setGoal(Lift.Goal.DOWN);
            robot.claw.setCurrentGoal(Claw.Goal.RESET);

        }
        sleep(3000);



//        robot.drive.followTrajectory(traj7);
//        sleep(200);
//        robot.drive.followTrajectory(traj8);

    }
}

