package org.firstinspires.ftc.teamcode.tests.necessary;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.buttons.StickyButton;
import org.firstinspires.ftc.teamcode.components.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.components.Claw;
import org.firstinspires.ftc.teamcode.components.Lift;

/**
 * This is a simple teleop routine for debugging your motor configuration.
 * Pressing each of the buttons will power its respective motor.
 *
 * Button Mappings:
 *
 * Xbox/PS4 Button - Motor
 *   X / ▢         - Front Left
 *   Y / Δ         - Front Right
 *   B / O         - Rear  Right
 *   A / X         - Rear  Left
 *                                    The buttons are mapped to match the wheels spatially if you
 *                                    were to rotate the gamepad 45deg°. x/square is the front left
 *                    ________        and each button corresponds to the wheel as you go clockwise
 *                   / ______ \
 *     ------------.-'   _  '-..+              Front of Bot
 *              /   _  ( Y )  _  \                  ^
 *             |  ( X )  _  ( B ) |     Front Left   \    Front Right
 *        ___  '.      ( A )     /|       Wheel       \      Wheel
 *      .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
 *     |       |                 |                      \
 *      '.___.' '.               |          Rear Left    \   Rear Right
 *               '.             /             Wheel       \    Wheel
 *                \.          .'              (A/X)        \   (B/O)
 *                  \________/
 *
 * Uncomment the @Disabled tag below to use this opmode.
 */
@Config
@TeleOp(group = "drive")
public class v4bTest extends LinearOpMode {

    private StickyButton leftServoIncStickyButton = new StickyButton();
    private StickyButton leftServoDecStickyButton = new StickyButton();
    private StickyButton rightServoIncStickyButton = new StickyButton();
    private StickyButton rightServoDecStickyButton = new StickyButton();

    private StickyButton leftServoMiniIncStickyButton = new StickyButton();
    private StickyButton leftServoMiniDecStickyButton = new StickyButton();
    private StickyButton rightServoMiniIncStickyButton = new StickyButton();
    private StickyButton rightServoMiniDecStickyButton = new StickyButton();

    private StickyButton leftServoResetStickyButton = new StickyButton();
    private StickyButton rightServoResetStickyButton = new StickyButton();

    private double leftServoPos = 0.0 ;
    private double rightServoPos = 1.0 ;

    private void mapControls() {
        leftServoIncStickyButton.update(gamepad1.dpad_up);
        leftServoPos += leftServoIncStickyButton.getState() ? 0.05 : 0;

        leftServoDecStickyButton.update(gamepad1.dpad_down);
        leftServoPos -= leftServoDecStickyButton.getState() ? 0.05 : 0;

        leftServoMiniIncStickyButton.update(gamepad1.dpad_right);
        leftServoPos += leftServoMiniIncStickyButton.getState() ? 0.01 : 0;

        leftServoMiniDecStickyButton.update(gamepad1.dpad_left);
        leftServoPos -= leftServoMiniDecStickyButton.getState() ? 0.01 : 0;

        leftServoResetStickyButton.update(gamepad1.left_bumper);
        leftServoPos = leftServoResetStickyButton.getState() ? 0.0 : leftServoPos;

        rightServoIncStickyButton.update(gamepad1.y);
        rightServoPos += rightServoIncStickyButton.getState() ? 0.05 : 0;

        rightServoDecStickyButton.update(gamepad1.a);
        rightServoPos -= rightServoDecStickyButton.getState() ? 0.05 : 0;

        rightServoMiniIncStickyButton.update(gamepad1.b);
        rightServoPos += rightServoMiniIncStickyButton.getState() ? 0.01 : 0;

        rightServoMiniDecStickyButton.update(gamepad1.x);
        rightServoPos -= rightServoMiniDecStickyButton.getState() ? 0.01 : 0;

        rightServoResetStickyButton.update(gamepad1.right_bumper);
        rightServoPos = rightServoMiniDecStickyButton.getState() ? 1.0 : rightServoPos;

        leftServoPos = Range.clip(leftServoPos,0.0,1.0);
        rightServoPos = Range.clip(rightServoPos,0.0,1.0);
    }
    
    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();

        Claw claw = new Claw(hardwareMap);

        telemetry.addLine("Press play to begin the debugging opmode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();

        while (!isStopRequested()) {
            mapControls();

            telemetry.addLine("Press buttons to adjust servo position");
            telemetry.addLine();
            telemetry.addLine("Xbox/PS4 Button - Action");
            telemetry.addLine("  X / ▢   : +0.01 Right");
            telemetry.addLine("  B / O   : -0.01 Right");
            telemetry.addLine("  Y / Δ   : +0.05 Right");
            telemetry.addLine("  A / X   : -0.05 Right");
            telemetry.addLine("dpad_right: +0.01 Left");
            telemetry.addLine("dpad_left : -0.01 Left");
            telemetry.addLine("dpad_up   : +0.05 Left");
            telemetry.addLine("dpad_down : -0.05 Left");
            telemetry.addLine();
            telemetry.addData("Right Pos:", rightServoPos);
            telemetry.addData("Left Pos: ", leftServoPos);
            telemetry.addLine();

            claw.setLeftFlipPosition(leftServoPos);
            claw.setRightFlipPosition(rightServoPos);
                    
            telemetry.update();
        }
    }
}
