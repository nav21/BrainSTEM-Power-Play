package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autonomous.cancellers.TimerCanceller;
import org.firstinspires.ftc.teamcode.autonomous.enums.LiftPosition;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.utils.Component;

/**
 * Created by parvs on 11/9/2018.
 */

public class Lift implements Component {
    public enum Goal {
        OPEN_LOOP, UP, DOWN
    }

    public enum Mode {
        LOW, HIGH, MED, JUNC
    }

    // PORTME Use REV PID
    //private static final double HOLD_LIFT_POWER = 0.15;
    private int currentPosition;
    // PORTME
    private static final int restPosition = 200; //FILL THIS IN; //TODO SET THESE VALUES
    private static final int junctionPosition = 3000; //FILL THIS IN;
    private static final int lowPosition = 10000;//FILL THIS IN;
    private static final int medPosition = 22000;//FILL THIS IN;
    private static final int highPosition = 34000;//FILL THIS IN;

    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;
    //private final Encoder liftEncoder;
    // private final RevMagnetSensor liftLimitSwitch;

    //TODO: adjust these values maybe
    private TimerCanceller resetLiftTimerCanceller = new TimerCanceller(750);
    private TimerCanceller extendLiftTimerCanceller = new TimerCanceller(500);

    private Goal goal = Goal.OPEN_LOOP;
    private Mode mode = Mode.HIGH;

    public Lift(HardwareMap map) {
        fl = map.dcMotor.get("flandperpendicularEncoder");
        fr = map.dcMotor.get("frandparallelEncoder");
        bl = map.dcMotor.get("blandliftEncoder");
        br = map.dcMotor.get("br");
       // liftEncoder = new Encoder(map.get(DcMotorEx.class, "blandliftEncoder"));
        // liftLimitSwitch = new RevMagnetSensor(map.get(DigitalChannel.class, "liftLimitSwitch"));

        // PORTME
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        //bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void initAuto() {
    }

    public void initBlockAuto() {

    }

    @Override
    public void initTeleOp() {
    }

    @Override
    public void update() {
        switch (goal) {
            case OPEN_LOOP:
                break;
            case UP:
                if (mode.equals(Mode.HIGH)) {
                    runLiftToPosition(LiftPosition.HIGH);
                } else if (mode.equals(Mode.MED)) {
                    runLiftToPosition(LiftPosition.MED);
                } else if (mode.equals(Mode.LOW)) {
                    runLiftToPosition(LiftPosition.LOW);
                } else if (mode.equals(Mode.JUNC)) {
                    runLiftToPosition(LiftPosition.JUNC);
                }
                break;
            case DOWN:
                runLiftToPosition(LiftPosition.REST);
                break;
        }

    }

    public void setGoal(Goal goal) {
        if (this.goal != goal) {
            this.goal = goal;
            resetLiftTimerCanceller.reset();
            extendLiftTimerCanceller.reset();
        }
    }

    public Goal getGoal() {
        return goal;
    }


    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public Mode getMode() {
        return mode;
    }

    public String test() {
        String failures = "";

        return failures;
    }

    public void setMotorPowers(double vFL, double vFR, double vBL, double vBR) {
        fl.setPower(vFL);
        fr.setPower(vFR);
        bl.setPower(vBL);
        br.setPower(vBR);
    }


    public void holdOrStopLift() {
        /*  PORTME
        if (liftLimitSwitch.isTriggered()) {
            liftMotor1.setPower(0);
            liftMotor2.setPower(0);
            liftMotor3.setPower(0);
            liftMotor4.setPower(0);
        } else {
            liftMotor1.setPower(HOLD_LIFT_POWER);
            liftMotor2.setPower(HOLD_LIFT_POWER);
            liftMotor3.setPower(HOLD_LIFT_POWER);
            liftMotor4.setPower(HOLD_LIFT_POWER);
        }
        */
    }

    public boolean getLimitSwtichState() {
        // return liftLimitSwitch.isTriggered();
        return false;
    }

    // PORTME
    public double getLiftEncoderTicks() {
        return bl.getCurrentPosition();
    }

    // PORTME
    public void runLiftToPosition(LiftPosition position) {
        switch (position) {
            case REST:
                bl.setTargetPosition(restPosition);
                bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setMotorPowers(-0.9, -0.9, -0.9, -0.9);
                if (restPosition == bl.getCurrentPosition()) {
                    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                break;
            case JUNC:
                bl.setTargetPosition(junctionPosition);
                bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setMotorPowers(1, 1, 1, 1);
//                if (junctionPosition == bl.getCurrentPosition()) {
//                    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                }
                break;
            case LOW:
                bl.setTargetPosition(lowPosition);
                bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setMotorPowers(1, 1, 1, 1);
//                if (lowPosition == bl.getCurrentPosition()) {
//                    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                }
                break;
            case MED:
                bl.setTargetPosition(medPosition);
                bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setMotorPowers(1, 1, 1, 1);
//                if (medPosition == bl.getCurrentPosition()) {
//                    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                }
                break;
            case HIGH:
                bl.setTargetPosition(highPosition);
                bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setMotorPowers(1, 1, 1, 1);
//                if (highPosition == bl.getCurrentPosition()) {
//                    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                }
                break;
        }
    }

}