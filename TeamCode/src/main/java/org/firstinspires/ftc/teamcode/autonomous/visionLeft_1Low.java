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
@Autonomous(name="Left_V+1Low", group = "auto")
public class visionLeft_1Low extends BaseAuto {
    private Trajectory goToMedGoalPosition ;
    private Trajectory depositPreloadLowGoal;
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
            xModifier *= -1.0;
            yModifier *= 1.0;
        } else {
            d = -1.0;
            xModifier *= 1.0;
            yModifier *= 1.0;
        }
        Pose2d startPose = new Pose2d(-36, d*64.6, Math.toRadians(d*90));
        robot.drive.setPoseEstimate(startPose);

        goToMedGoalPosition = robot.drive.trajectoryBuilder(startPose,true)
                .lineToSplineHeading(new Pose2d(-36, d*36, Math.toRadians(0)), BMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToConstantHeading(new Vector2d(-36, d*25), BMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID))
                .addTemporalMarker(0.7, () -> robot.lift.setGoal(Lift.Goal.UP))
                .addTemporalMarker(2, () -> robot.claw.setCurrentGoal(Claw.Goal.FLIP))
                .build();
        depositPreloadLowGoal = robot.drive.trajectoryBuilder(goToMedGoalPosition.end())
                .lineTo(new Vector2d(-43+xModifier, d*(25+yModifier)), BMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        goToFirstConeAndGetReadyForPark1 = robot.drive.trajectoryBuilder(depositPreloadLowGoal.end())
                .lineToConstantHeading(new Vector2d(-36, d*25), BMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.5, () -> robot.lift.setGoal(Lift.Goal.DOWN))
                .build();
        strafeForPark = robot.drive.trajectoryBuilder(goToFirstConeAndGetReadyForPark1.end())
                .lineToConstantHeading(new Vector2d(-36, d*36.5), BMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        parkA = robot.drive.trajectoryBuilder(strafeForPark.end())
                .lineTo(new Vector2d(-60, d*36.5), BMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.lift.setGoal(Lift.Goal.DOWN))
                .addTemporalMarker(0.4, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
                .build();
        parkB = robot.drive.trajectoryBuilder(strafeForPark.end())
                .lineTo(new Vector2d(-14, d*36.5), BMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
        robot.lift.setHeight(Lift.Height.LOW);
        robot.drive.followTrajectoryAsync(goToMedGoalPosition);
        CheckWait(0);         // FollowTrajectory
        CheckWait(200);

        robot.drive.followTrajectoryAsync(depositPreloadLowGoal);
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