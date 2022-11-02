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

public abstract class BaseAuto extends LinearOpMode {
    public static double DISTANCE = 24; // inches
    AutoBrainSTEMRobot robot=null;

    private void CheckWait(boolean checkDrive, boolean callUpdates, double minMS, double maxMS) {

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
                return;
            }

            // Update the drive
            if (checkDrive) {
                robot.drive.update();
            }

            // Update the shooterPID
            if (callUpdates) {
                // It's possible that your 'Auto' class already handles this with threading but I'm not sure.
                // It might be better to just define an 'update()' function in the AutoBrainSTEMRobot class and call that.
                // robot.update();
            }

            // Check timer expiration, bail if too long
            if (maxMS < now) {
                return;
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
            return;
        }
    }

    @Override
    public void runOpMode() {
        PhotonCore.enable();
        robot = new AutoBrainSTEMRobot(this);
        VisionLibrary visionLibrary = new VisionLibrary(this);

        robot.start();

        robot.initBlockAuto();
        visionLibrary.init();

        SignalSleevePosition signalSleevePosition = SignalSleevePosition.UNKNOWN;


        while (!opModeIsActive() && !isStopRequested()) {

           signalSleevePosition = visionLibrary.getSignalSleevePosition();

            telemetry.addData("Status", "Waiting...");
         telemetry.addData("Signal Pos", signalSleevePosition);
          telemetry.update();
        }

        visionLibrary.stopVision();

        if (isStopRequested()) return;

        runMain(robot, signalSleevePosition);

        robot.stop();
    }

    public abstract void runMain(AutoBrainSTEMRobot robot, SignalSleevePosition signalSleevePosition);
}
