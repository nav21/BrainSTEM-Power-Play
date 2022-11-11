package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
@Autonomous(name="Right_V+1", group = "auto")
public class visionRight_1 extends BaseAuto {
    private Trajectory goToMedGoalPosition ;
    private Trajectory depositPreloadMedGoal;
    private Trajectory goToFirstConeAndGetReadyForPark1;
    private Trajectory strafeForPark;
    private Trajectory park1;
    private Trajectory park3;
    private Trajectory parkB;
    private Trajectory parkA;

    public void buildPaths(AutoBrainSTEMRobot robot) {
        String className = this.getClass().getSimpleName().toLowerCase();
        double d;

        // Left/right autos, only difference is sign and parking spot
        if (className.contains("right")) {
            d = 1.0;
        } else {
            d = -1.0;
        }
        Pose2d startPose = new Pose2d(-36, d*64.6, Math.toRadians(d*90));

        robot.drive.setPoseEstimate(startPose);
        goToMedGoalPosition = robot.drive.trajectoryBuilder(startPose,true)
                .lineToSplineHeading(new Pose2d(-36, d*36, Math.toRadians(d*180)), BMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToConstantHeading(new Vector2d(-36, d*25), BMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID))
                .addTemporalMarker(0.7, () -> robot.lift.setGoal(Lift.Goal.UP))
                .addTemporalMarker(2, () -> robot.claw.setCurrentGoal(Claw.Goal.FLIP))
                .build();
        depositPreloadMedGoal = robot.drive.trajectoryBuilder(goToMedGoalPosition.end())
                .lineTo(new Vector2d(-32, d*25), BMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        goToFirstConeAndGetReadyForPark1 = robot.drive.trajectoryBuilder(depositPreloadMedGoal.end())
                .lineToConstantHeading(new Vector2d(-37, d*25), BMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.5, () -> robot.lift.setGoal(Lift.Goal.DOWN))
                .build();
        strafeForPark = robot.drive.trajectoryBuilder(goToFirstConeAndGetReadyForPark1.end())
                .lineToConstantHeading(new Vector2d(-37, d*36), BMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        parkA = robot.drive.trajectoryBuilder(strafeForPark.end())
                .lineTo(new Vector2d(-60, d*36), BMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.lift.setGoal(Lift.Goal.DOWN))
                .addTemporalMarker(0.4, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
                .build();
        parkB = robot.drive.trajectoryBuilder(strafeForPark.end())
                .lineTo(new Vector2d(-14, d*36), BMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.lift.setGoal(Lift.Goal.DOWN))
                .addTemporalMarker(0.4, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
                .build();

        if (className.contains("right")) {
            park1 = parkB;
            park3 = parkA;
        } else {
            park1 = parkA;
            park3 = parkB;
        }
    }

    public void runMain(AutoBrainSTEMRobot robot, SignalSleevePosition signalSleevePosition) {
        robot.lift.setGoal(Lift.Goal.OPEN_LOOP);
        robot.claw.setCurrentGoal(Claw.Goal.OPEN_LOOP);
        robot.lift.setMode(Lift.Mode.MED);
        robot.drive.followTrajectoryAsync(goToMedGoalPosition);
        CheckWait(0);         // FollowTrajectory
        CheckWait(200);

        robot.drive.followTrajectoryAsync(depositPreloadMedGoal);
        CheckWait(0);        // FollowTrajectory

        robot.claw.setCurrentGoal(Claw.Goal.RELEASE);
        CheckWait(350);

        robot.claw.setCurrentGoal(Claw.Goal.RETURN_MID);
        CheckWait(400);

        robot.drive.followTrajectoryAsync(goToFirstConeAndGetReadyForPark1);
        CheckWait(0);        // FollowTrajectory
        CheckWait(750);

        robot.drive.followTrajectoryAsync(strafeForPark);
        CheckWait(0);        // FollowTrajectory
        CheckWait(1000);

        if (signalSleevePosition == SignalSleevePosition.ONE) {
            robot.drive.followTrajectoryAsync(park1);
            CheckWait(0);        // FollowTrajectory
        } else if (signalSleevePosition == SignalSleevePosition.THREE){
            robot.drive.followTrajectoryAsync(park3);
            CheckWait(0);        // FollowTrajectory
        }

        robot.lift.setGoal(Lift.Goal.DOWN);
        robot.claw.setCurrentGoal(Claw.Goal.RESET);
        CheckWait(3000);    // Sleep
    }
}