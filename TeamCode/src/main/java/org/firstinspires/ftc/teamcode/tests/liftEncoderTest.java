package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.enums.ClawPosition;
import org.firstinspires.ftc.teamcode.autonomous.enums.LiftPosition;
import org.firstinspires.ftc.teamcode.buttons.StickyButton;
import org.firstinspires.ftc.teamcode.components.BrainSTEMRobot;

@TeleOp
public class liftEncoderTest extends LinearOpMode {

    private double MOTOR_TICK_COUNT;

    @Override
    public void runOpMode() {
        // Initialize a new robot object
        BrainSTEMRobot robot = new BrainSTEMRobot(this);

        robot.initTeleOp();

        while (!opModeIsActive() && !isStopRequested()) {
            //Status to show if telemetry was initialized
            telemetry.addData("Status", "Initialized");
            telemetry.update();

        }


        while (opModeIsActive()) {
            // PORTME
            // MOTOR_TICK_COUNT = robot.lift.getLiftEncoderTicks();
            telemetry.addData("Deposit Mode", MOTOR_TICK_COUNT);
            telemetry.update();
        }
    }
}
