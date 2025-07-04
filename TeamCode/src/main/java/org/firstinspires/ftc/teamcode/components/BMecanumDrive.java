package org.firstinspires.ftc.teamcode.components;

import static org.firstinspires.ftc.teamcode.components.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.components.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.components.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.components.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.components.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.components.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.components.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.components.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.components.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.components.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.components.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.autonomous.enums.ClawPosition;
import org.firstinspires.ftc.teamcode.autonomous.enums.LiftPosition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.utils.BotLog;
import org.firstinspires.ftc.teamcode.utils.Component;
import org.firstinspires.ftc.teamcode.util.AxisDirection;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class BMecanumDrive extends MecanumDrive implements Component {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(3, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(3, 0, 0);
    public static double localizer = 1;
    // TODO
    //public static double LATERAL_MULTIPLIER = 1.68;
    public static double LATERAL_MULTIPLIER = 1.161/(60.5/59.5);

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private BNO055IMU imu;
    public VoltageSensor batteryVoltageSensor;
    // public BotLog logger = new BotLog();

    public void initIMU(HardwareMap hardwareMap) {
        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.POS_Z);
    }

    public BMecanumDrive(HardwareMap hardwareMap) {
        this(hardwareMap, 0.25);
    }

    public BMecanumDrive(HardwareMap hardwareMap, double timeout) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(2.0)), timeout);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        initIMU(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        //
        //
//        if (localizer == 1){
//            setLocalizer(new TwoWheelTrackingLocalizer(hardwareMap, this));
//        }
//        else{
//
//        }

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

        // logger.LOGLEVEL = logger.LOGDEBUG ;
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    @Override
    public void initAuto() {

    }

    @Override
    public void initTeleOp() {

    }

    @Override
    public void updateComponent() {
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turnAsync(double angle, double VEL, double ACCEL) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                new TrajectorySequenceBuilder(getPoseEstimate(), VEL_CONSTRAINT, ACCEL_CONSTRAINT, VEL, ACCEL )
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }
    public void getEncoderPositions (){

    }
    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    // TODO -- Need this for complicated autos
    //public void breakFollowing() {
    //    trajectorySequenceRunner.breakFollowing();
    //}

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void reset() {

    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    @Override
    public String test() {
        return null;
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }


    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double vLF, double vLR, double vRR, double vRF) {
        leftFront.setPower(vLF);
        leftRear.setPower(vLR);
        rightRear.setPower(vRR);
        rightFront.setPower(vRF);
    }

    /*
    public double getQHeading()
    {
        Quaternion q = imu.getQuaternionOrientation();
        Orientation gyroOrien = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        q = q.normalized();

        // This code was leveraged from here:
        // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/

        // May not need the next 3 lines
        double t=q.x*q.y + q.z*q.w;
        double h,a,b;
        double sqy = q.y * q.y;
        //double b;
        double sqx = q.x * q.x;
        double sqz = q.z * q.z;

        // We only need 'b' here since our bot is only intending to rotate in one dimension
        // We don't need to worry about singularities for Quaternion to Euler conversion
        // Because of a single dimension of turning.
        // May not need h and a
        h = Math.atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * sqy - 2 * sqz);
        a = Math.asin(2 * t);
        b = Math.atan2(2 * q.x * q.w - 2 * q.y * q.z, 1 - 2 * sqx - 2 * sqz);

        // This code is a way to compare our Quaternion vs Euler gyro readings
        logger.logD("MechLog",String.format(" getQHeading: t:%f, h/Y:%f(%f), a/Z:%f(%f), b/X:%f(%f)",
                     t,
                     -(h/Math.PI)*180,
                     AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.secondAngle),
                     -(a/Math.PI)*180,
                     AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.firstAngle),
                     -(b/Math.PI)*180,
                     AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.thirdAngle)));

        return (-(b/Math.PI)*180.0);
    }
    */

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
