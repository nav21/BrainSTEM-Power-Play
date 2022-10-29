package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.autonomous.enums.ClawPosition;
import org.firstinspires.ftc.teamcode.autonomous.enums.LiftPosition;
import org.firstinspires.ftc.teamcode.buttons.StickyButton;
import org.firstinspires.ftc.teamcode.buttons.ToggleButton;
import org.firstinspires.ftc.teamcode.components.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.SmoothDrive;
import org.firstinspires.ftc.teamcode.components.Claw;
import org.firstinspires.ftc.teamcode.components.Lift;

@TeleOp
public class BrainSTEMTeleOp extends LinearOpMode {

    // private double leftTuningFactor = 1;
    // private double rightTuningFactor = 1;
    // private double driveInterpolationFactor = 2;
    private static final double THRESHOLD = 0.001;

    /*
    private ToggleButton collectorToggle = new ToggleButton();
    private ToggleButton carouselToggle = new ToggleButton();
    private ToggleButton liftToggle = new ToggleButton();
    private ToggleButton depositorGateToggle = new ToggleButton();
    private ToggleButton extensionLiftToggle = new ToggleButton();
    private ToggleButton collectorGateToggle = new ToggleButton();

    private StickyButton depositorStickyButton = new StickyButton();

    private ToggleButton modeToggleButton = new ToggleButton();
    */

    ////////////
    //DRIVER 1//
    ////////////
    /*
        NEW DRIVER 1 CONTROLS
        Left Stick X:
        Left Stick Y: Drive left side
        Left Stick Button: Press left button once to enable collector flip servos and move it to exactly horizontal, press again to disable flip servos
        Right Stick X:
        Right Stick Y: Drive right side
        Right Stick Button:
        D-pad Up:
        D-pad Left:
        D-pad Down: Carousel left wheel move , click again to spin other way
        D-pad Right:
        Start:
        X: Press X once to close the gate servo, extend the depositor out, and deposit the element.
        Press X again to close the deposit servo, retract depositor, and open gate servo.
        B: Press B once to close the depositor gate servo, and press B once more to open it
        Y:
        A: Press A once to put collector gate servo in and click A again to put it out.
        Left Bumper: Reverse collector when holding
        Left Trigger: Move lift down
        Right Bumper: Toggle collector on/off
        Right Trigger: Move lift up
     */
    // private double drive;
    // private double turn;


    private ElapsedTime runtime = new ElapsedTime();

    private boolean collectorOn;
    private boolean collectorTransfer;

    private boolean carouselReverse;
    private boolean toggleCarouselOn;

    private boolean collectorGateOut;

    private double moveLiftUp;
    private double moveLiftDown;

    private boolean collectorGate;

    private boolean resetDepositorToggle;
    private int depositorToggleHits = 0;

    private boolean modeToggle;


    private void mapControls() {
        // drive = Math.pow(Math.abs(gamepad1.left_stick_y), driveInterpolationFactor) * Math.signum(-gamepad1.left_stick_y)/2.25;
        // turn = (Math.pow(Math.abs(gamepad1.right_stick_x), driveInterpolationFactor) * Math.signum(-gamepad1.right_stick_x))/3.5;


        collectorOn = gamepad1.right_bumper;
        collectorTransfer = gamepad1.left_bumper;

        carouselReverse = gamepad1.dpad_up;

        collectorGateOut = gamepad1.y;
        collectorGate = gamepad1.b;

        moveLiftUp = gamepad1.right_trigger;
        moveLiftDown = gamepad1.left_trigger;

       // resetDepositorToggle = gamepad1.right_stick_button;
//        depositorStickyButton.update(gamepad1.left_stick_button);
//        depositorToggleHits += depositorStickyButton.getState() ? 1 : 0;
//
//        modeToggle = modeToggleButton.update(gamepad1.x);

    }

    @Override
    public void runOpMode() {
        // Initialize a new robot object
        BrainSTEMRobot robot = new BrainSTEMRobot(this);

        robot.initTeleOp();

        // This is a 'better' way to do the smooth drive where we push everything into a class
        SmoothDrive sd = new SmoothDrive( gamepad1, gamepad2 );

        while (!opModeIsActive() && !isStopRequested()) {
            //Status to show if telemetry was initialized
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }

        runtime.reset();
        while (opModeIsActive()) {
            mapControls();

            // This is a 'better' way to do the smooth drive where we push everything into a class
            sd.update(Math.toDegrees(robot.drive.getRawExternalHeading()));
            robot.drive.setMotorPowers(sd.l_f_motor_power, sd.l_b_motor_power, sd.r_b_motor_power, sd.r_f_motor_power);


            if(moveLiftUp > THRESHOLD) {
                robot.lift.setMotorPowers(moveLiftUp, -moveLiftUp, -moveLiftUp, moveLiftUp);
                telemetry.addLine("Running Motor: Front Left");
            } else if(moveLiftDown > THRESHOLD) {
                robot.lift.setMotorPowers(-moveLiftDown, moveLiftDown, moveLiftDown, -moveLiftDown);
                telemetry.addLine("Running Motor: Rear Left");
            } else {
                robot.lift.setMotorPowers(0.05, -0.05, -0.05, 0.05);
                telemetry.addLine("Running Motor: None");
            }
            /*
            //If the x value of the left stick, the y value of the left stick, or the x value of
            //the right stick is greater than the THRESHOLD value, move the robot
            if ((Math.abs(turn) > THRESHOLD) || Math.abs(drive) > THRESHOLD) {
                robot.drive.setMotorPowers(drive- turn, drive+turn);
            } else {
                robot.drive.setMotorPowers(0, 0);
            }
            */

            /*
            if (collectorOn) {
                robot.collector.setCurrentGoal(Collector.Goal.COLLECT);
            }

            if (carouselReverse) {
                robot.collector.setCollectorPower(-0.46);
            } else if (toggleCarouselOn) {
                robot.collector.setCollectorPower(0.46);
            } else if (collectorTransfer) {
                robot.collector.setCollectorPower(1);
            } else if (robot.collector.getCurrentGoal() == Collector.Goal.OPEN_LOOP) {
                robot.collector.setCollectorPower(0);
            }

            if (moveLiftUp > THRESHOLD) {
                robot.depositor.setLiftPower(moveLiftUp);
            } else if (moveLiftDown > THRESHOLD) {
                robot.depositor.setLiftPower(-moveLiftDown);
            } else {
                robot.depositor.holdOrStopLift();
            }

            if (resetDepositorToggle) {
                robot.depositor.setGoal(Depositor.Goal.IN);
                depositorToggleHits = 0;
            }

            if (collectorGate) {
                robot.collector.setGateServoPosition(CollectorPosition.IN);
            }
            if (collectorGateOut) {
                robot.collector.setGateServoPosition(CollectorPosition.OUT);
            }

            if (depositorToggleHits == 1) {
                robot.depositor.setGoal(Depositor.Goal.UP);
            } else if (depositorToggleHits >= 2) {
                robot.depositor.setGoal(Depositor.Goal.DEPOSIT);
            }

            if (modeToggle) {
                robot.depositor.setMode(Depositor.Mode.HIGH);
            } else {
                robot.depositor.setMode(Depositor.Mode.LOW);
            }

            robot.claw.update();

            robot.lift.update();

            telemetry.addData("Deposit Mode", robot.depositor.getMode());
            telemetry.addData("Lift Limit Switch", robot.depositor.getLimitSwtichState());
            */
            telemetry.update();
        }
    }
}
