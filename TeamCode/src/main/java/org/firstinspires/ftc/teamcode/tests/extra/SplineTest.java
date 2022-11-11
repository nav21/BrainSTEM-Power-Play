package org.firstinspires.ftc.teamcode.tests.extra;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.BMecanumDrive;
import org.firstinspires.ftc.teamcode.components.DriveConstants;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")

public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BMecanumDrive drive = new BMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose = new Pose2d(-36, -64.6, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        Trajectory goToMedGoalPosition = drive.trajectoryBuilder(startPose,true)
                .lineToSplineHeading(new Pose2d(-36, -24, Math.toRadians(-180)), BMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addTemporalMarker(0.5, () -> robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID))
//                .addTemporalMarker(1.1, () -> robot.lift.setGoal(Lift.Goal.UP))
//                .addTemporalMarker(2, () -> robot.claw.setCurrentGoal(Claw.Goal.FLIP))
                .build();
        Trajectory depositPreloadMedGoal = drive.trajectoryBuilder(goToMedGoalPosition.end())
                .lineTo(new Vector2d(-30, -24), BMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addTemporalMarker(1.5, () -> robot.claw.setCurrentGoal(Claw.Goal.RELEASE))
//                .addTemporalMarker(2, () -> robot.claw.setCurrentGoal(Claw.Goal.RETURN_MID))
//                .addTemporalMarker(2.5, () -> robot.lift.setGoal(Lift.Goal.DOWN))
                .build();
        Trajectory goToFirstConeAndGetReadyForPark1 = drive.trajectoryBuilder(depositPreloadMedGoal.end())
                .lineToConstantHeading(new Vector2d(-36, -24), BMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory goToFirstConeAndGetReadyForPark2 = drive.trajectoryBuilder(goToFirstConeAndGetReadyForPark1.end())
                .lineToConstantHeading(new Vector2d(-42, -12), BMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectory(goToMedGoalPosition);
        sleep(200);
        drive.followTrajectory(depositPreloadMedGoal);
        sleep(200);
        drive.followTrajectory(goToFirstConeAndGetReadyForPark1);
        sleep(1);
        drive.followTrajectory(goToFirstConeAndGetReadyForPark2);


//        robot.drive.followTrajectory(traj7);
//        sleep(200);
//        robot.drive.followTrajectory(traj8);

    }
}
