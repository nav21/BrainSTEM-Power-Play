package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.vision.FFVisionLibrary;
import org.firstinspires.ftc.teamcode.autonomous.vision.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.buttons.StickyButton;
import org.firstinspires.ftc.teamcode.components.AutoBrainSTEMRobot;

public abstract class BaseAuto extends LinearOpMode {
    public static double DISTANCE = 24; // inches


    @Override
    public void runOpMode() {
        AutoBrainSTEMRobot robot = new AutoBrainSTEMRobot(this);
        FFVisionLibrary ffVisionLibrary = new FFVisionLibrary(this);

        robot.start();

        robot.initBlockAuto();
        ffVisionLibrary.init();

        TeamMarkerPosition teamMarkerPosition = TeamMarkerPosition.UNKNOWN;

        /*
        StickyButton hsvwMinusStickyButton = new StickyButton();
        StickyButton hsvwPlusStickyButton = new StickyButton();
        StickyButton hsvwValuesUpStickyButton = new StickyButton();
        StickyButton hsvwValuesDownStickyButton = new StickyButton();

        String hsvwString = "height";
        int[] hsvwValues = new int[]{120, 200, 60, 180, 125, 120, 255, 255};
        int hsvwCounter = 0;

        while (!opModeIsActive() && !isStopRequested()) {
            hsvwMinusStickyButton.update(gamepad1.dpad_left);
            hsvwCounter -= hsvwMinusStickyButton.getState() ? 1 : 0;

            hsvwPlusStickyButton.update(gamepad1.dpad_right);
            hsvwCounter += hsvwPlusStickyButton.getState() ? 1 : 0;

            if (hsvwCounter == 0) {
                hsvwString = "height";
            }

            if (hsvwCounter == 1) {
                hsvwString = "width";
            }

            if (hsvwCounter == 2) {
                hsvwString = "lower_hue";
            }

            if (hsvwCounter == 3) {
                hsvwString = "lower_saturation";
            }

            if (hsvwCounter == 4) {
                hsvwString = "lower_value";
            }

            if (hsvwCounter == 5) {
                hsvwString = "upper_hue";
            }

            if (hsvwCounter == 6) {
                hsvwString = "upper_saturation";
            }

            if (hsvwCounter == 7) {
                hsvwString = "upper_value";
            }

            hsvwValuesUpStickyButton.update(gamepad1.dpad_up);
            hsvwValuesDownStickyButton.update(gamepad1.dpad_down);

            if (hsvwValuesUpStickyButton.getState()) {
                hsvwValues[hsvwCounter] += 5;
            }

            if (hsvwValuesDownStickyButton.getState()) {
                hsvwValues[hsvwCounter] -= 5;
            }

            if (hsvwCounter == 8) {
                hsvwCounter = 0;
            }

            if (hsvwCounter == -1) {
                hsvwCounter = 7;
            }

            ffVisionLibrary.teamMarkerDetector.update(AllianceColor.RED, hsvwValues[2], hsvwValues[3], hsvwValues[4],
                    hsvwValues[5], hsvwValues[6], hsvwValues[7]);
            ffVisionLibrary.teamMarkerDetector.setDesiredHeight(hsvwValues[0]);
            ffVisionLibrary.teamMarkerDetector.setDesiredWidth(hsvwValues[1]);

            teamMarkerPosition = ffVisionLibrary.teamMarkerDetector.getTeamMarkerPosition();

            telemetry.addData("Status", "Waiting...");
            telemetry.addData("Team Marker Pos", teamMarkerPosition);
            telemetry.addData("Currently changing", hsvwString);
            telemetry.addData("Current value", hsvwValues[hsvwCounter]);
            telemetry.addData("Height", hsvwValues[0]);
            telemetry.addData("Width", hsvwValues[1]);
            telemetry.addData("Lower Hue", hsvwValues[2]);
            telemetry.addData("Lower Saturation", hsvwValues[3]);
            telemetry.addData("Lower Value", hsvwValues[4]);
            telemetry.addData("Higher Hue", hsvwValues[5]);
            telemetry.addData("Higher Saturation", hsvwValues[6]);
            telemetry.addData("Higher Value", hsvwValues[7]);
            telemetry.addData("Areas", ffVisionLibrary.teamMarkerDetector.getAreas());

            telemetry.update();
        }
        */

        ffVisionLibrary.stopVision();

        if (isStopRequested()) return;

        runMain(robot, teamMarkerPosition);

        robot.stop();
    }

    public abstract void runMain(AutoBrainSTEMRobot robot, TeamMarkerPosition teamMarkerPosition);
}
