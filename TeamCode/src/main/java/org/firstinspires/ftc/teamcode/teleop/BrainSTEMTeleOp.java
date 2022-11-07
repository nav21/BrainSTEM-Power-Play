package org.firstinspires.ftc.teamcode.teleop;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.autonomous.cancellers.TimerCanceller;
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
*/
    private StickyButton heightIncStickyButton = new StickyButton();
    private StickyButton heightDecStickyButton = new StickyButton();


    private StickyButton depositorIncStickyButton = new StickyButton();
    private StickyButton depositorDecStickyButton = new StickyButton();
    //private ToggleButton depositorToggleButton = new ToggleButton();
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
    private double MOTOR_TICK_COUNT;


    private boolean carouselReverse;
    private boolean toggleCarouselOn;

    private boolean modeButton;
    private boolean collectorGateOut;

    private boolean moveLiftUp;
    private boolean moveLiftDown;

    private double moveLiftUpTrigger;
    private double moveLiftDownTrigger;

    private boolean collectorGate;
    private boolean depositor;

    private boolean modeThing;
    private boolean resetDepositorToggle;
    private int depositorToggleHits = 0;

    //private boolean modeToggle;
    private int heightToggleHits=3;
    private int mode = 0;



    private void mapControls(BrainSTEMRobot robot) {
        if( robot.claw.isSafeToChangeClawGoal()) {
            depositorIncStickyButton.update(gamepad1.a);
            depositorToggleHits += depositorIncStickyButton.getState() ? 1 : 0;

            depositorDecStickyButton.update(gamepad1.b);
            if (depositorDecStickyButton.getState()) {
                if (depositorToggleHits == 1) {
                    depositorToggleHits = 5;
                } else {
                    depositorToggleHits = Math.max(0, depositorToggleHits - 1);
                }
            }
        }

        if( robot.claw.isSafeToChangeClawGoal() && ((robot.claw.getCurrentGoal() == Claw.Goal.RETURN_MID) || (robot.claw.getCurrentGoal() == Claw.Goal.COLLECT_MID))) {
            moveLiftUp = gamepad1.right_bumper;
            moveLiftDown = gamepad1.left_bumper;

            heightIncStickyButton.update(gamepad1.y);
            heightToggleHits += heightIncStickyButton.getState() ? 1 : 0;

            heightDecStickyButton.update(gamepad1.x);
            heightToggleHits -= heightDecStickyButton.getState() ? 1 : 0;
            Range.clip(heightToggleHits, 0, 3);
        }
    }

    @Override
    public void runOpMode() {
        PhotonCore.enable();

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
            mapControls(robot);

            // This is a 'better' way to do the smooth drive where we push everything into a class
            sd.update(Math.toDegrees(robot.drive.getRawExternalHeading()));
            robot.drive.setMotorPowers(sd.l_f_motor_power, sd.l_b_motor_power, sd.r_b_motor_power, sd.r_f_motor_power);

            if(moveLiftUp) {
                robot.lift.setGoal(Lift.Goal.UP);
                telemetry.addLine("Running Motaaor: Front Left");
            } else if(moveLiftDown) {
                robot.lift.setGoal(Lift.Goal.DOWN);
                telemetry.addLine("Running a: Rear Left");
            }

            //if (resetDepositorToggle) {
            //    robot.claw.setCurrentGoal(Claw.Goal.RETURN);
            //    depositorToggleHits = 0;
            //}

            switch (depositorToggleHits % 6) {
                case 1:
                    robot.claw.setCurrentGoal(Claw.Goal.COLLECT_MID);
                    break;
                case 2:
                    robot.claw.setCurrentGoal(Claw.Goal.FLIP);
                    break;
                case 3:
                    robot.claw.setCurrentGoal(Claw.Goal.RELEASE);
                    break;
                case 4:
                    robot.claw.setCurrentGoal(Claw.Goal.RETURN_MID);
                    break;
                case 5:
                    robot.claw.setCurrentGoal(Claw.Goal.RESET);
                    depositorToggleHits = 0;
                    break;
                default:
                    break;
            }

            switch (heightToggleHits % 4) {
                case 0:
                    robot.lift.setMode(Lift.Mode.JUNC);
                    break;
                case 1:
                    robot.lift.setMode(Lift.Mode.LOW);
                    break;
                case 2:
                    robot.lift.setMode(Lift.Mode.MED);
                    break;
                case 3:
                    robot.lift.setMode(Lift.Mode.HIGH);
                    break;
            }

            robot.claw.updateComponent();
            robot.lift.updateComponent();

            telemetry.addData("Lift Mode", robot.lift.getMode());
            telemetry.addData("Claw Goal", robot.claw.getCurrentGoal());
            MOTOR_TICK_COUNT = robot.lift.getLiftEncoderTicks();
            telemetry.addData("Mode:", mode);
            telemetry.addData("depositorToggleHits: ", depositorToggleHits);
            telemetry.addData("Lift Height: ", MOTOR_TICK_COUNT);
            telemetry.addData("Lift pwr: ", robot.lift.pwr);
            telemetry.update();
        }
    }
}
