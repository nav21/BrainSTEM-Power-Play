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
    // private static final double THRESHOLD = 0.001;

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

    // Mech drive related variables
    private int[] speedIdx = new int[] {0, 0};
    private  double[] speedModifier = new double[] {0.75, 0.90};
    private boolean[] FOD = new boolean[] {false, false};
    private double[] forward = new double[2], strafe = new double[2], rotate = new double[2];
    private double[] prevForward = new double[2], prevStrafe = new double[2], prevRotate = new double[2];
    private double[] prevTime = new double[2];
    private double maxForwardChange=4.0, maxRotateChange=5.0, maxStrafeChange=3.0;
    private boolean smoothDrive = true;
    private double now;
    private double deltaT;

    private double l_f_motor_power;
    private double l_b_motor_power;
    private double r_f_motor_power;
    private double r_b_motor_power;
    private double maxPwr = 0.0;
 
    private double theta, r_speed, new_x, new_y, degrees;
    private double[] adjustAngle = {0,0};
    private ElapsedTime runtime = new ElapsedTime();
    /*
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
    */

    private void mapControls() {
        // drive = Math.pow(Math.abs(gamepad1.left_stick_y), driveInterpolationFactor) * Math.signum(-gamepad1.left_stick_y)/2.25;
        // turn = (Math.pow(Math.abs(gamepad1.right_stick_x), driveInterpolationFactor) * Math.signum(-gamepad1.right_stick_x))/3.5;

        /*
        collectorOn = gamepad1.right_bumper;
        collectorTransfer = gamepad1.left_bumper;

        toggleCarouselOn = carouselToggle.update(gamepad1.dpad_down);
        carouselReverse = gamepad1.dpad_up;

        collectorGateOut = gamepad1.y;
        collectorGate = gamepad1.b;

        moveLiftUp = gamepad1.right_trigger;
        moveLiftDown = gamepad1.left_trigger;

        resetDepositorToggle = gamepad1.right_stick_button;
        depositorStickyButton.update(gamepad1.left_stick_button);
        depositorToggleHits += depositorStickyButton.getState() ? 1 : 0;

        modeToggle = modeToggleButton.update(gamepad1.x);
        */
    }

    private void driveControls(BrainSTEMRobot robot, Gamepad gamepad, int padIdx) {

        // Read the controller 1 (driver) stick positions
        strafe[padIdx] = gamepad.left_stick_x;
        forward[padIdx] = -gamepad.left_stick_y;
        rotate[padIdx] = gamepad.right_stick_x;
 
        /* High Precision Mode is 25% power */
        if( gamepad.left_stick_button ) {
            strafe[padIdx] *= .25;
            forward[padIdx] *= .25;
        }
 
        if( gamepad.right_stick_button ) {
            rotate[padIdx] *= .25;
        }
 
        // Convert to FOD as soon as possible so all the smooth drive code applies correctly
        // This code is terrible.
        // Beware the function calls that pass information in and out via global vars
        if (FOD[padIdx]) {
            // Get the current robot heading
            degrees = Math.toDegrees(robot.drive.getRawExternalHeading());
 
            if (FOD[padIdx]) {
                // Convert the X/Y Cartesion for strafe and forward into Polar
                CarToPol(strafe[padIdx], forward[padIdx]);
                // Rotate the Polar coordinates by the robot's heading
                theta -= AngleUnit.DEGREES.normalize(degrees - adjustAngle[padIdx]);
                // Convert the new Polar back into Cartesian
                PolToCar(r_speed);
                // Replace the strafe and forward power with translated values
                strafe[padIdx] = new_x;
                forward[padIdx] = new_y;
                // Now the robot moves in orientation of the field
            }
        }
 
        // A little bump to make strafe/rotate feel more responsive
        strafe[padIdx] *= 1.25;
        rotate[padIdx] *= 1.5;

        if(smoothDrive) {
            now = runtime.seconds();
            deltaT = now - prevTime[padIdx];
 
            if ((prevStrafe[padIdx] != 0) && (strafe[padIdx] != 0) && (Math.signum(prevStrafe[padIdx]) != Math.signum(strafe[padIdx]))) {
                strafe[padIdx] = 0;
            }
            if ((prevForward[padIdx] != 0) && (forward[padIdx] != 0) && (Math.signum(prevForward[padIdx]) != Math.signum(forward[padIdx]))) {
                forward[padIdx] = 0;
            }
            if ((prevRotate[padIdx] != 0) && (rotate[padIdx] != 0) && (Math.signum(prevRotate[padIdx]) != Math.signum(rotate[padIdx]))) {
                rotate[padIdx] = 0;
            }

            if (Math.abs(strafe[padIdx]) > 0.20) {
                if (prevStrafe[padIdx] < strafe[padIdx]) {
                    strafe[padIdx] = Math.min(strafe[padIdx], prevStrafe[padIdx] + (maxStrafeChange * deltaT));
                } else {
                    strafe[padIdx] = Math.max(strafe[padIdx], prevStrafe[padIdx] - (maxStrafeChange * deltaT));
                }
            }
            if (Math.abs(forward[padIdx]) > 0.20) {
                if (prevForward[padIdx] < forward[padIdx]) {
                    forward[padIdx] = Math.min(forward[padIdx], prevForward[padIdx] + (maxForwardChange * deltaT));
                } else {
                    forward[padIdx] = Math.max(forward[padIdx], prevForward[padIdx] - (maxForwardChange * deltaT));
                }
            }
            if (Math.abs(rotate[padIdx]) > 0.20) {
                if (prevRotate[padIdx] < rotate[padIdx]) {
                    rotate[padIdx] = Math.min(rotate[padIdx], prevRotate[padIdx] + (maxRotateChange * deltaT));
                } else {
                    rotate[padIdx] = Math.max(rotate[padIdx], prevRotate[padIdx] - (maxRotateChange * deltaT));
                }
            }
            prevTime[padIdx] = now;
            prevStrafe[padIdx] = strafe[padIdx];
            prevForward[padIdx] = forward[padIdx];
            prevRotate[padIdx] = rotate[padIdx];
 
            // Remove 15% deadzone
            if (strafe[padIdx] >= 0.025) {
                strafe[padIdx] = (strafe[padIdx] * 0.85) + 0.15;
            }
            if (forward[padIdx] >= 0.025) {
                forward[padIdx] = (forward[padIdx] * 0.85) + 0.15;
            }
            if (rotate[padIdx] >= 0.025) {
                rotate[padIdx] = (rotate[padIdx] * 0.85) + 0.15;
            }
            if (strafe[padIdx] <= -0.025) {
                strafe[padIdx] = (strafe[padIdx] * 0.85) - 0.15;
            }
            if (forward[padIdx] <= -0.025) {
                forward[padIdx] = (forward[padIdx] * 0.85) - 0.15;
            }
            if (rotate[padIdx] <= -0.025) {
                rotate[padIdx] = (rotate[padIdx] * 0.85) - 0.15;
            }
        }

        // Rotate a little left
        if (gamepad.dpad_left) {
            rotate[padIdx] -= 0.4;
        }
        // Rotate a little right
        if (gamepad.dpad_right) {
            rotate[padIdx] += 0.4;
        }
 
        // This adds the powers from both controllers together scaled for each controller and FOD
        l_f_motor_power = ((forward[padIdx] + strafe[padIdx] + rotate[padIdx]) * speedModifier[speedIdx[padIdx]]);
        l_b_motor_power = ((forward[padIdx] - strafe[padIdx] + rotate[padIdx]) * speedModifier[speedIdx[padIdx]]);
        r_f_motor_power = ((forward[padIdx] - strafe[padIdx] - rotate[padIdx]) * speedModifier[speedIdx[padIdx]]);
        r_b_motor_power = ((forward[padIdx] + strafe[padIdx] - rotate[padIdx]) * speedModifier[speedIdx[padIdx]]);
 
        if(smoothDrive) {
            // Find the largest power request ignoring sign
            maxPwr = Math.max(Math.max(Math.max(Math.abs(l_f_motor_power), Math.abs(l_b_motor_power)),
                    Math.abs(r_f_motor_power)), Math.abs(r_b_motor_power));
 
            // If this is greater than 1.0, need to scale everything back equally
            // Max is now guaranteed positive which is good to reduce magnitude without changing sign
            // Now the power is scaled and limited to range of {-1.0, 1.0)
            if (maxPwr > 1.0) {
                l_f_motor_power /= maxPwr;
                l_b_motor_power /= maxPwr;
                r_f_motor_power /= maxPwr;
                r_b_motor_power /= maxPwr;
            }
        }

        // We follow different logic based on whether we are in manual driver control or switch
        // control to the automatic mode
        robot.drive.setMotorPowers(l_f_motor_power, l_b_motor_power, r_b_motor_power, r_f_motor_power);
    }


    @Override
    public void runOpMode() {
        // Initialize a new robot object
        BrainSTEMRobot robot = new BrainSTEMRobot(this);

        robot.initTeleOp();

        // TODO
        // This is a 'better' way to do the smooth drive where we push everything into a class
        // SmoothDrive sd = new SmoothDrive( gamepad1 );

        while (!opModeIsActive() && !isStopRequested()) {
            //Status to show if telemetry was initialized
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }

        runtime.reset();
        while (opModeIsActive()) {
            mapControls();
            driveControls( robot, gamepad1, 1 );

            // TODO
            // This is a 'better' way to do the smooth drive where we push everything into a class
            // sd.update(Math.toDegrees(robot.drive.getRawExternalHeading()));
            // robot.drive.setMotorPowers(sd.l_f_motor_power, sd.l_b_motor_power, sd.r_b_motor_power, sd.r_f_motor_power);

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

    private void CarToPol(double x, double y) {
        r_speed = Math.sqrt(x * x + y * y);
        theta = Math.atan2(y, x);
 
        //theta to degrees
        theta = theta * 180 / 3.14159265358979323;
    }
 
    private void PolToCar(double r) {
        theta = theta / 180 * 3.14159265358979323;
        new_x = r * Math.cos(theta);
        new_y = r * Math.sin(theta);
    }

}
