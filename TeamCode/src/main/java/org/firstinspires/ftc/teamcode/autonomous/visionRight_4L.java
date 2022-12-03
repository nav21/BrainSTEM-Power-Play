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
@Autonomous(name="Right_V+4L", group = "auto")
public class visionRight_4L extends BaseAuto {
    private Trajectory goToPoleMed_1;
    private Trajectory goToPoleMed_2;
    private Trajectory goToCone5_1;
    private Trajectory goToCone5_2;
    private Trajectory goToCone5_3;
    private Trajectory goToPoleLow;
    private Trajectory goToCone4;
    private Trajectory goToCone3;
    private Trajectory goToCone2;
    private Trajectory goToCone1;
    private Trajectory getInPositionForPark_1;
    private Trajectory getInPositionForPark_2;
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

        double coneStack_X = -64.5;
        double coneStack_Y = d*13.25;

        Pose2d startPose = new Pose2d(-36, d*64, Math.toRadians(d*90));
        robot.drive.setPoseEstimate(startPose);

        goToPoleMed_1 = robot.drive.trajectoryBuilder(startPose,true)
                .lineToSplineHeading(new Pose2d(-36, d*36, Math.toRadians(d*180)), BMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToConstantHeading(new Vector2d(-36, d*25), BMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID))
                .addTemporalMarker(0.7, () -> robot.lift.setGoal(Lift.Goal.UP))
                .addTemporalMarker(2, () -> robot.claw.setCurrentGoal(Claw.Goal.FLIP))
                .build();
        goToPoleMed_2 = robot.drive.trajectoryBuilder(goToPoleMed_1.end())
                .lineTo(new Vector2d(-30.5+xModifier, d*(25+yModifier)), BMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        goToCone5_1 = robot.drive.trajectoryBuilder(goToPoleMed_2.end())
                .lineToConstantHeading(new Vector2d(-37, d*25), BMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.5, () -> robot.lift.setHeight(Lift.Height.CONE_5))
                .build();
        goToCone5_2 = robot.drive.trajectoryBuilder(goToCone5_1.end())
                .lineToConstantHeading(new Vector2d(-40, coneStack_Y), BMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        goToCone5_3 = robot.drive.trajectoryBuilder(goToCone5_2.end())
                .lineToConstantHeading(new Vector2d( coneStack_X, coneStack_Y), BMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        goToPoleLow = robot.drive.trajectoryBuilder(goToCone5_3.end())
                .lineToSplineHeading(new Pose2d(-51.5, d*21, Math.toRadians(d*225)), BMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.01, () -> robot.lift.setHeight(Lift.Height.LOW))
                .addTemporalMarker(0.6, () -> robot.claw.setCurrentGoal(Claw.Goal.FLIP))
                .build();
        goToCone4 = robot.drive.trajectoryBuilder(goToPoleLow.end())
                .lineToSplineHeading(new Pose2d( coneStack_X, coneStack_Y, Math.toRadians(d*180)), BMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.5, () -> robot.lift.setHeight(Lift.Height.CONE_4))
                .build();
        goToCone3 = robot.drive.trajectoryBuilder(goToPoleLow.end())
                .lineToSplineHeading(new Pose2d( coneStack_X, coneStack_Y, Math.toRadians(d*180)), BMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.5, () -> robot.lift.setHeight(Lift.Height.CONE_3))
                .build();
        goToCone2 = robot.drive.trajectoryBuilder(goToPoleLow.end())
                .lineToSplineHeading(new Pose2d( coneStack_X, coneStack_Y, Math.toRadians(d*180)), BMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.5, () -> robot.lift.setHeight(Lift.Height.CONE_2))
                .build();
        goToCone1 = robot.drive.trajectoryBuilder(goToPoleLow.end())
                .lineToSplineHeading(new Pose2d( coneStack_X, coneStack_Y, Math.toRadians(d*180)), BMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.5, () -> robot.lift.setHeight(Lift.Height.REST))
                .build();
        getInPositionForPark_1= robot.drive.trajectoryBuilder(goToPoleLow.end()) // TODO change this end trajectoy
                .lineToSplineHeading(new Pose2d(-59, d*18, Math.toRadians(d*180)), BMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.5, () -> robot.lift.setGoal(Lift.Goal.DOWN))
                .build();
        getInPositionForPark_2 = robot.drive.trajectoryBuilder(getInPositionForPark_1.end()) // TODO change this end trajectoy
                .lineToConstantHeading(new Vector2d(-60, d*38), BMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        parkA = robot.drive.trajectoryBuilder(getInPositionForPark_2.end())
                .lineTo(new Vector2d(-12, d*38), BMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.5, () -> robot.lift.setGoal(Lift.Goal.DOWN))
                .addTemporalMarker(0.4, () -> robot.claw.setCurrentGoal(Claw.Goal.RESET))
                .build();
        parkB = robot.drive.trajectoryBuilder(getInPositionForPark_2.end())
                .lineTo(new Vector2d(-36, d*38), BMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.5, () -> robot.lift.setGoal(Lift.Goal.DOWN))
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
        String className = this.getClass().getSimpleName().toLowerCase();

        robot.lift.setGoal(Lift.Goal.OPEN_LOOP);
        robot.claw.setCurrentGoal(Claw.Goal.OPEN_LOOP);
        robot.lift.setHeight(Lift.Height.MED);
        robot.drive.followTrajectoryAsync(goToPoleMed_1);
        CheckWait(0);         // FollowTrajectory

        robot.drive.followTrajectoryAsync(goToPoleMed_2);
        CheckWait(0);         // FollowTrajectory

        robot.claw.setCurrentGoal(Claw.Goal.DEPOSIT);

        robot.drive.followTrajectoryAsync(goToCone5_1);
        CheckWait(0);         // FollowTrajectory

        robot.drive.followTrajectoryAsync(goToCone5_2);
        CheckWait(0);         // FollowTrajectory

        robot.drive.followTrajectoryAsync(goToCone5_3);
        CheckWait(0);         // FollowTrajectory

        robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID);
        CheckWait(350);

        robot.drive.followTrajectoryAsync(goToPoleLow);
        CheckWait(0);         // FollowTrajectory

        robot.claw.setCurrentGoal(Claw.Goal.DEPOSIT);
        CheckWait(100);

        robot.drive.followTrajectoryAsync(goToCone4);
        CheckWait(0);         // FollowTrajectory

        robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID);
        CheckWait(350);

        robot.drive.followTrajectoryAsync(goToPoleLow);
        CheckWait(0);         // FollowTrajectory

        robot.claw.setCurrentGoal(Claw.Goal.DEPOSIT);

        robot.drive.followTrajectoryAsync(goToCone3);
        CheckWait(0);         // FollowTrajectory

        robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID);
        CheckWait(0);         // FollowTrajectory
        CheckWait(350);

        robot.drive.followTrajectoryAsync(goToPoleLow);
        CheckWait(0);         // FollowTrajectory

        robot.claw.setCurrentGoal(Claw.Goal.DEPOSIT);

        robot.drive.followTrajectoryAsync(goToCone2);
        CheckWait(0);         // FollowTrajectory

        robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID);
        CheckWait(0);         // FollowTrajectory
        CheckWait(350);

        robot.drive.followTrajectoryAsync(goToPoleLow);
        CheckWait(0);         // FollowTrajectory

        robot.claw.setCurrentGoal(Claw.Goal.DEPOSIT);

        if ((className.contains("left") && (signalSleevePosition == SignalSleevePosition.ONE)) ||
                (className.contains("right") && (signalSleevePosition == SignalSleevePosition.THREE))) {

            robot.drive.followTrajectoryAsync(goToCone1);
            CheckWait(0);         // FollowTrajectory

            robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID);
            CheckWait(0);         // FollowTrajectory
            CheckWait(350);

            robot.drive.followTrajectoryAsync(goToPoleLow);
            CheckWait(0);         // FollowTrajectory

            robot.claw.setCurrentGoal(Claw.Goal.DEPOSIT);

            robot.drive.followTrajectoryAsync(getInPositionForPark_1);
            CheckWait(0);         // FollowTrajectory

            robot.lift.setGoal(Lift.Goal.DOWN);
            robot.claw.setCurrentGoal(Claw.Goal.RESET);

        } else {

            robot.drive.followTrajectoryAsync(getInPositionForPark_1);
            CheckWait(0);         // FollowTrajectory

            robot.drive.followTrajectoryAsync(getInPositionForPark_2);
            CheckWait(0);         // FollowTrajectory

            robot.lift.setGoal(Lift.Goal.DOWN);
            robot.claw.setCurrentGoal(Claw.Goal.RESET);

            if (className.contains("right")) {
                if (signalSleevePosition == SignalSleevePosition.ONE) {
                    robot.drive.followTrajectoryAsync(park2);
                    CheckWait(0);         // FollowTrajectory

                } else if (signalSleevePosition == SignalSleevePosition.TWO) {
                    robot.drive.followTrajectoryAsync(park3);
                    CheckWait(0);         // FollowTrajectory
                }
            } else {
                if (signalSleevePosition == SignalSleevePosition.TWO) {
                    robot.drive.followTrajectoryAsync(park2);
                    CheckWait(0);         // FollowTrajectory

                } else if (signalSleevePosition == SignalSleevePosition.THREE) {
                    robot.drive.followTrajectoryAsync(park3);
                    CheckWait(0);         // FollowTrajectory
                }
            }
        }

        CheckWait(3000);    // Sleep
    }
}