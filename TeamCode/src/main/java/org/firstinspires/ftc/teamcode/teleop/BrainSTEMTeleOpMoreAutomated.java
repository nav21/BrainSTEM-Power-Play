package org.firstinspires.ftc.teamcode.teleop;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.autonomous.enums.ClawPosition;
import org.firstinspires.ftc.teamcode.buttons.StickyButton;
import org.firstinspires.ftc.teamcode.buttons.ToggleButton;
import org.firstinspires.ftc.teamcode.components.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.components.Claw;
import org.firstinspires.ftc.teamcode.components.Lift;
import org.firstinspires.ftc.teamcode.utils.SmoothDrive;

@TeleOp
public class BrainSTEMTeleOpMoreAutomated extends LinearOpMode {

    // private double leftTuningFactor = 1;
    // private double rightTuningFactor = 1;
    // private double driveInterpolationFactor = 2;
    private static final double THRESHOLD = 0.001;

    private StickyButton heightIncStickyButton = new StickyButton();
    private StickyButton heightDecStickyButton = new StickyButton();

    private StickyButton clawIncStickyButton = new StickyButton();
    private StickyButton clawDecStickyButton = new StickyButton();

    private StickyButton zeroLiftStickyButton = new StickyButton();
    ////////////
    //DRIVER 1//
    ////////////
    /*
        NEW DRIVER 1 CONTROLS
        Left Stick X:
        Left Stick Y: Drive left side
        Left Stick Button:
        Right Stick X:
        Right Stick Y: Drive right side
        Right Stick Button:
        D-pad Up:
        D-pad Left:
        D-pad Down:
        D-pad Right:
        Start:
        X:
        B:
        Y:
        A:
        Left Bumper:
        Left Trigger:
        Right Bumper:
        Right Trigger:
     */
    // private double drive;
    // private double turn;

    private ElapsedTime runtime = new ElapsedTime();

    private boolean moveLiftUp;
    private boolean moveLiftDown;
    private boolean clawButton;

    private int clawToggleHits = 0;

    private int heightToggleHits=7;

    private void mapControls(BrainSTEMRobot robot) {
        zeroLiftStickyButton.update(gamepad1.dpad_down);

        if( robot.claw.isSafeToChangeClawGoal()) {
            clawIncStickyButton.update(gamepad1.a);
            clawToggleHits += clawIncStickyButton.getState() ? 1 : 0;

            clawDecStickyButton.update(gamepad1.b);
            if (clawDecStickyButton.getState()) {
                if (clawToggleHits == 1) {
                    clawToggleHits = 3;
                } else {
                    clawToggleHits = Math.max(0, clawToggleHits - 1);
                }
            }
        }

//        if( robot.claw.isSafeToChangeClawGoal() && ((robot.claw.getCurrentGoal() == Claw.Goal.RETURN_MID) || (robot.claw.getCurrentGoal() == Claw.Goal.COLLECT_MID))) {
          moveLiftUp = gamepad1.right_bumper;
           moveLiftDown = gamepad1.left_bumper;
//        }

//        if((robot.lift.getGoal() == Lift.Goal.DOWN) || ( robot.claw.isSafeToChangeClawGoal() && ((robot.claw.getCurrentGoal() == Claw.Goal.RETURN_MID) || (robot.claw.getCurrentGoal() == Claw.Goal.COLLECT_MID)))) {
            heightIncStickyButton.update(gamepad1.y);
            heightToggleHits += heightIncStickyButton.getState() ? 1 : 0;

            heightDecStickyButton.update(gamepad1.x);
            heightToggleHits -= heightDecStickyButton.getState() ? 1 : 0;
            Range.clip(heightToggleHits, 0, 7);
//        }
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

        // Hardcode some init here
        robot.claw.initClaw();
        robot.claw.setClawServoPosition(ClawPosition.OPEN);
        robot.lift.zeroLift();

        runtime.reset();
        while (opModeIsActive()) {
            mapControls(robot);

            if( zeroLiftStickyButton.getState()) {
                robot.drive.setMotorPowers(0,0,0,0);
                robot.lift.zeroLift();
            }

            // This is a 'better' way to do the smooth drive where we push everything into a class
            sd.update(Math.toDegrees(robot.drive.getRawExternalHeading()));
            robot.drive.setMotorPowers(sd.l_f_motor_power, sd.l_b_motor_power, sd.r_b_motor_power, sd.r_f_motor_power);

            if(moveLiftUp) {
                robot.lift.setGoal(Lift.Goal.UP);
                telemetry.addLine("Running Motor: Front Left");
            } else if(moveLiftDown) {
                robot.lift.setGoal(Lift.Goal.DOWN);
                telemetry.addLine("Running a: Rear Left");
            }

            switch (clawToggleHits % 4) {
                case 1:
                    robot.claw.setCurrentGoal(Claw.Goal.COLLECT_25);
                    break;
                case 2:
                    robot.claw.setCurrentGoal(Claw.Goal.COLLECT);
                    break;
                case 3:
                    robot.claw.setCurrentGoal(Claw.Goal.DEPOSIT);
                    clawToggleHits = 0;
                    break;
                default:
                    break;
            }

            switch (heightToggleHits % 8) {
                case 0:
                    robot.lift.setMode(Lift.Mode.JUNC);
                    break;
                case 1:
                    robot.lift.setMode(Lift.Mode.CONE_2);
                    break;
                case 2:
                    robot.lift.setMode(Lift.Mode.CONE_3);
                    break;
                case 3:
                    robot.lift.setMode(Lift.Mode.CONE_4);
                    break;
                case 4:
                    robot.lift.setMode(Lift.Mode.CONE_5);
                    break;
                case 5:
                    robot.lift.setMode(Lift.Mode.LOW);
                    break;
                case 6:
                    robot.lift.setMode(Lift.Mode.MED);
                    break;
                case 7:
                    robot.lift.setMode(Lift.Mode.HIGH);
                    break;
            }

            robot.claw.updateComponent();
            robot.lift.updateComponent();

            telemetry.addData("clawToggleHits: ", clawToggleHits);
            telemetry.addData("Claw Goal", robot.claw.getCurrentGoal());
            telemetry.addData("Lift Mode", robot.lift.getMode());
            telemetry.addData("Act Height: ", robot.lift.getLiftEncoderTicks());
            telemetry.addData("Tgt Height: ", robot.lift.getTgtPos());
            telemetry.addData("Lift pwr: ", robot.lift.pwr);
            telemetry.addData("Min pwr: ", robot.lift.pid.getOutputMin());
            telemetry.addData("Max pwr: ", robot.lift.pid.getOutputMax());
            telemetry.update();
        }
    }
}
