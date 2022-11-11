package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.vision.VisionLibrary;
import org.firstinspires.ftc.teamcode.autonomous.vision.SignalSleevePosition;
import org.firstinspires.ftc.teamcode.autonomous.vision.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.buttons.StickyButton;
import org.firstinspires.ftc.teamcode.components.AutoBrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.Component;

public abstract class BaseAuto extends LinearOpMode {
    AutoBrainSTEMRobot robot=null;
    public boolean useThreads = true;

    @Override
    public void runOpMode() {
        PhotonCore.enable();

        telemetry.addLine("Creating Robot class");
        telemetry.update();
        robot = new AutoBrainSTEMRobot(this);

        telemetry.addLine("Creating Vision Library");
        telemetry.update();
        VisionLibrary visionLibrary = new VisionLibrary(this);

        if(useThreads) {
            telemetry.addLine("Starting threads");
            telemetry.update();
            robot.start();
        }

        telemetry.addLine("InitAuto for components");
        telemetry.update();
        robot.initAuto();

        telemetry.addLine("Building paths");
        telemetry.update();
        buildPaths(robot);

        telemetry.addLine("Initializing Vision Library");
        telemetry.update();
        visionLibrary.init();

        SignalSleevePosition signalSleevePosition = SignalSleevePosition.TWO;
        SignalSleevePosition signalSleevePositionTemp;

        while (!opModeIsActive() && !isStopRequested()) {
            // Never forget the last valid thing we saw.
            signalSleevePositionTemp = visionLibrary.getSignalSleevePosition();
            if (signalSleevePositionTemp != SignalSleevePosition.UNKNOWN) {
                signalSleevePosition =  signalSleevePositionTemp;
            }

            telemetry.addData("Status", "Waiting...");
            telemetry.addData("Signal Pos", signalSleevePosition);
            telemetry.update();
        }

        visionLibrary.stopVision();

        if (isStopRequested()) return;

        runMain(robot, signalSleevePosition);

        if(useThreads) {
            robot.stop();
        }
    }

    public abstract void buildPaths(AutoBrainSTEMRobot robot);
    public abstract void runMain(AutoBrainSTEMRobot robot, SignalSleevePosition signalSleevePosition);

    // This routine will manage drive and component updates and keep time
    // It is meant to fulfill the roles of the FollowTrajectory(), sleep(), and threading
    // If threading is enabled, component update is skipped
    // This routine will monitor the drive for idle for a min of minMS and max of maxMS
    // We return the drive status on exit in case the drive was not done by our timeout -- this
    // might be considered an error and we might consider shutting auto down.
    // If the drive is idle, the minMS can be thought of as a sleep that updates components.
    public boolean CheckWait(double minMS) { return(CheckWait(minMS, minMS)); }
    public boolean CheckWait(double minMS, double maxMS) {
        boolean checkDrive = true;
        boolean updateComponents = !useThreads;
        NanoClock localClock = NanoClock.system();
        double now = localClock.seconds();

        // Convert to seconds
        minMS /= 1000;
        maxMS /= 1000;

        minMS += now;
        if (maxMS > 0) {
            maxMS += now;
        } else {
            maxMS = Double.POSITIVE_INFINITY;
        }

        while (opModeIsActive()) {
            // Get the time
            now = localClock.seconds();

            // Master stop
            if (!opModeIsActive()) {
                return(robot.drive.isBusy());
            }

            // Update the drive
            if (checkDrive) {
                robot.drive.update();
            }

            // Update the components
            if (updateComponents) {
                robot.updateComponents();
            }

            // Check timer expiration, bail if too long
            if (maxMS < now) {
                return(robot.drive.isBusy());
            }

            // Make sure to wait for the minimum time
            if (minMS > now) {
                continue;
            }

            // Drive still running? Wait for it.
            if (checkDrive) {
                if (robot.drive.isBusy()) {
                    continue;
                }
            }
            // No reason to be here (past the minMS timer, drive is idle)
            return (robot.drive.isBusy());
        }
        return (robot.drive.isBusy());
    }
}
