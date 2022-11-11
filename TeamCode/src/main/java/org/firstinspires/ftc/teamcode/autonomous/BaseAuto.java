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
    AutoBrainSTEMRobot robot=null;

    @Override
    public void runOpMode() {
        PhotonCore.enable();

        telemetry.addLine("Creating Robot class");
        telemetry.update();
        robot = new AutoBrainSTEMRobot(this);

        telemetry.addLine("Creating Vision Library");
        telemetry.update();
        VisionLibrary visionLibrary = new VisionLibrary(this);

        telemetry.addLine("Starting threads");
        telemetry.update();
        robot.start();

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

        robot.stop();
    }

    public abstract void buildPaths(AutoBrainSTEMRobot robot);
    public abstract void runMain(AutoBrainSTEMRobot robot, SignalSleevePosition signalSleevePosition);
}
