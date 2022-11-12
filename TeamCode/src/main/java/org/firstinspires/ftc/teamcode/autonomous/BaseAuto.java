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
    public boolean useThreads = false;

    private StickyButton awayStickyButton = new StickyButton();
    private StickyButton nearStickyButton = new StickyButton();
    private StickyButton leftStickyButton = new StickyButton();
    private StickyButton rightStickyButton = new StickyButton();
    private StickyButton goStickyButton = new StickyButton();

    public double xModifier = 0.0;
    public double yModifier = 0.0;

    private SignalSleevePosition signalSleevePosition = SignalSleevePosition.TWO;


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

        showTelemetry();

        goStickyButton.update(gamepad1.a);
        while ((goStickyButton.getState()==false) && !opModeIsActive() && !isStopRequested()) {
            if (isStopRequested()) return;
            awayStickyButton.update(gamepad1.dpad_up);
            yModifier += awayStickyButton.getState() ? 0.5 : 0.0;
            nearStickyButton.update(gamepad1.dpad_down);
            yModifier -= nearStickyButton.getState() ? 0.5 : 0.0;

            leftStickyButton.update(gamepad1.dpad_left);
            xModifier -= leftStickyButton.getState() ? 0.5 : 0.0;
            rightStickyButton.update(gamepad1.dpad_right);
            xModifier += rightStickyButton.getState() ? 0.5 : 0.0;

            telemetry.addLine("Configure X/Y offsets.");
            telemetry.addData("XModifier: ", "%.1f (%s)", xModifier, (xModifier<0)?"Left":((xModifier>0)?"Right":"None"));
            telemetry.addData("YModifier: ", "%.1f (%s)", yModifier, (yModifier<0)?"Farther":((yModifier>0)?"Closer":"None"));
            telemetry.addLine("");
            telemetry.addLine("Press <a> to continue...");

            telemetry.update();

            goStickyButton.update(gamepad1.a);
        }

        if (isStopRequested()) return;

        telemetry.addLine("Building paths");
        telemetry.update();
        buildPaths(robot);

        telemetry.addLine("Initializing Vision Library");
        telemetry.update();
        visionLibrary.init();

        SignalSleevePosition signalSleevePositionTemp;

        while (!opModeIsActive() && !isStopRequested()) {
            // Never forget the last valid thing we saw.
            signalSleevePositionTemp = visionLibrary.getSignalSleevePosition();
            if (signalSleevePositionTemp != SignalSleevePosition.UNKNOWN) {
                signalSleevePosition =  signalSleevePositionTemp;
            }

            showTelemetry();
        }

        visionLibrary.stopVision();

        if (isStopRequested()) return;

        runMain(robot, signalSleevePosition);

        if(useThreads) {
            robot.stop();
        }
    }

    public void showTelemetry() {
        //telemetry.addData("clawToggleHits: ", clawToggleHits);
        telemetry.addData("Claw Goal", robot.claw.getCurrentGoal());
        telemetry.addData("Lift Mode", robot.lift.getMode());
        telemetry.addData("Act Height: ", robot.lift.getLiftEncoderTicks());
        telemetry.addData("Tgt Height: ", robot.lift.getTgtPos());
        telemetry.addData("Lift pwr: ", robot.lift.pwr);
        telemetry.addData("Min pwr: ", robot.lift.pid.getOutputMin());
        telemetry.addData("Max pwr: ", robot.lift.pid.getOutputMax());

        telemetry.addLine("");
        telemetry.addData("Status", "Waiting...");
        telemetry.addData("Signal Pos", signalSleevePosition);

        telemetry.update();
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

            showTelemetry();

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
