package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.vision.VisionLibrary;
import org.firstinspires.ftc.teamcode.autonomous.vision.SignalSleevePosition;
import org.firstinspires.ftc.teamcode.autonomous.vision.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.buttons.StickyButton;
import org.firstinspires.ftc.teamcode.components.AutoBrainSTEMRobot;

public abstract class BaseAuto extends LinearOpMode {
    public static double DISTANCE = 24; // inches


    @Override
    public void runOpMode() {
        AutoBrainSTEMRobot robot = new AutoBrainSTEMRobot(this);
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
