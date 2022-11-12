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
@Autonomous(name="Left_V+3", group = "auto")
public class visionLeft_2 extends BaseAuto {
    private Trajectory goToMedGoalPosition ;
    private Trajectory depositPreloadMedGoal;
    private Trajectory goToFirstCone1;
    private Trajectory goToFirstCone2;
    private Trajectory goToFirstCone3;
    private Trajectory goDepositLow1;
    private Trajectory goReturnToCone1;
    private Trajectory goReturnToCone2;
    private Trajectory getInPositionForPark1;
    private Trajectory getInPositionForPark2;
    private Trajectory park2;
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
                .lineToSplineHeading(new Pose2d(-36, d*36, Math.toRadians(d*180)), BMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToConstantHeading(new Vector2d(-36, d*25), BMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID))
                .addTemporalMarker(0.7, () -> robot.lift.setGoal(Lift.Goal.UP))
                .addTemporalMarker(2, () -> robot.claw.setCurrentGoal(Claw.Goal.FLIP))
                .build();
        depositPreloadMedGoal = robot.drive.trajectoryBuilder(goToMedGoalPosition.end())
                .lineTo(new Vector2d(-31+xModifier, d*(25+yModifier)), BMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        goToFirstCone1 = robot.drive.trajectoryBuilder(depositPreloadMedGoal.end())
                .lineToConstantHeading(new Vector2d(-37, d*25), BMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1, () -> robot.lift.setMode(Lift.Mode.CONE_5))
                .build();
        goToFirstCone2 = robot.drive.trajectoryBuilder(goToFirstCone1.end())
                .lineToConstantHeading(new Vector2d(-40, d*14), BMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        goToFirstCone3 = robot.drive.trajectoryBuilder(goToFirstCone2.end())
                .lineToConstantHeading(new Vector2d(-65, d*14), BMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
                .build();
        goDepositLow1 = robot.drive.trajectoryBuilder(goToFirstCone3.end())
                .lineToSplineHeading(new Pose2d(-52, d*21, Math.toRadians(d*225)), BMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.lift.setMode(Lift.Mode.LOW))
                .addTemporalMarker(0.6, () -> robot.claw.setCurrentGoal(Claw.Goal.FLIP))
                .build();
        goReturnToCone1 = robot.drive.trajectoryBuilder(goDepositLow1.end())
                .lineToSplineHeading(new Pose2d(-65, d*13.5, Math.toRadians(d*180)), BMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
                .addTemporalMarker(0.2, () -> robot.lift.setMode(Lift.Mode.CONE_4))
                .build();
        goReturnToCone2 = robot.drive.trajectoryBuilder(goDepositLow1.end())
                .lineToSplineHeading(new Pose2d(-65, d*13.5, Math.toRadians(d*180)), BMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
                .addTemporalMarker(0.2, () -> robot.lift.setMode(Lift.Mode.CONE_3))
                .build();
        getInPositionForPark1= robot.drive.trajectoryBuilder(goDepositLow1.end()) // TODO change this end trajectoy
                .lineToSplineHeading(new Pose2d(-59, d*18, Math.toRadians(d*180)), BMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.lift.setGoal(Lift.Goal.DOWN))
                .addTemporalMarker(0.3, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
                .build();
        getInPositionForPark2 = robot.drive.trajectoryBuilder(getInPositionForPark1.end()) // TODO change this end trajectoy
                .lineToConstantHeading(new Vector2d(-60, d*39), BMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        parkA = robot.drive.trajectoryBuilder(getInPositionForPark2.end())
                .lineTo(new Vector2d(-12, d*38), BMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        parkB = robot.drive.trajectoryBuilder(getInPositionForPark2.end())
                .lineTo(new Vector2d(-36, d*38), BMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.lift.setGoal(Lift.Goal.DOWN))
                .addTemporalMarker(0.4, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
                .build();

        if (className.contains("right")) {
            park2 = parkA;
            park3 = parkB;
        } else {
            park2 = parkB;
            park3 = parkA;
        }
    }

    public void runMain(AutoBrainSTEMRobot robot, SignalSleevePosition signalSleevePosition) {

        robot.lift.setGoal(Lift.Goal.OPEN_LOOP);
        robot.claw.setCurrentGoal(Claw.Goal.OPEN_LOOP);
        robot.lift.setMode(Lift.Mode.MED);
        robot.drive.followTrajectoryAsync(goToMedGoalPosition);
        CheckWait(0);         // FollowTrajectory
        CheckWait(100);

        robot.drive.followTrajectoryAsync(depositPreloadMedGoal);
        CheckWait(0);         // FollowTrajectory

        robot.claw.setCurrentGoal(Claw.Goal.RELEASE);
        CheckWait(200);

        robot.claw.setCurrentGoal(Claw.Goal.RETURN_MID);
        CheckWait(150);

        robot.drive.followTrajectoryAsync(goToFirstCone1);
        CheckWait(0);         // FollowTrajectory
        CheckWait(50);

        robot.drive.followTrajectoryAsync(goToFirstCone2);
        CheckWait(0);         // FollowTrajectory
        CheckWait(50);

        robot.drive.followTrajectoryAsync(goToFirstCone3);
        CheckWait(0);         // FollowTrajectory
        CheckWait(250);

        robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID);
        CheckWait(350);

        robot.drive.followTrajectoryAsync(goDepositLow1);
        CheckWait(0);         // FollowTrajectory
        CheckWait(100);

        robot.claw.setCurrentGoal(Claw.Goal.RELEASE);
        CheckWait(150);

        robot.claw.setCurrentGoal(Claw.Goal.RETURN_MID);
        CheckWait(350);

        robot.drive.followTrajectoryAsync(goReturnToCone1);
        CheckWait(0);         // FollowTrajectory
        CheckWait(350);

        robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID);
        CheckWait(350);

        robot.drive.followTrajectoryAsync(goDepositLow1);
        CheckWait(0);         // FollowTrajectory
        CheckWait(100);

        robot.claw.setCurrentGoal(Claw.Goal.RELEASE);
        CheckWait(0);         // FollowTrajectory
        CheckWait(150);

        robot.claw.setCurrentGoal(Claw.Goal.RETURN_MID);
        CheckWait(0);         // FollowTrajectory
        CheckWait(350);

        robot.drive.followTrajectoryAsync(goReturnToCone2);
        CheckWait(0);         // FollowTrajectory
        CheckWait(350);

        robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID);
        CheckWait(0);         // FollowTrajectory
        CheckWait(350);

        robot.drive.followTrajectoryAsync(goDepositLow1);
        CheckWait(0);         // FollowTrajectory
        CheckWait(100);

        robot.claw.setCurrentGoal(Claw.Goal.RELEASE);
        CheckWait(0);         // FollowTrajectory
        CheckWait(350);

        robot.claw.setCurrentGoal(Claw.Goal.RETURN_MID);
        CheckWait(0);         // FollowTrajectory
        CheckWait(350);

        robot.drive.followTrajectoryAsync(getInPositionForPark1);
        CheckWait(0);         // FollowTrajectory
        CheckWait(50);

        robot.drive.followTrajectoryAsync(getInPositionForPark2);
        CheckWait(0);         // FollowTrajectory
        CheckWait(50);



        if (signalSleevePosition == SignalSleevePosition.TWO) {
            robot.drive.followTrajectoryAsync(park2);
            CheckWait(0);         // FollowTrajectory

        } else if (signalSleevePosition == SignalSleevePosition.THREE){
            robot.drive.followTrajectoryAsync(park3);
            CheckWait(0);         // FollowTrajectory

        }

        robot.lift.setGoal(Lift.Goal.DOWN);
        robot.claw.setCurrentGoal(Claw.Goal.RESET);
        CheckWait(3000);    // Sleep
    }
}