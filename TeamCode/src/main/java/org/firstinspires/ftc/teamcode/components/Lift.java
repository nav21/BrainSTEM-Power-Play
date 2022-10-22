package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.cancellers.TimerCanceller;
import org.firstinspires.ftc.teamcode.autonomous.enums.LiftPosition;
import org.firstinspires.ftc.teamcode.utils.Component;
import org.firstinspires.ftc.teamcode.wrappers.RevMagnetSensor;

/**
 * Created by parvs on 11/9/2018.
 */

public class Lift implements Component {
    public enum Goal {
        OPEN_LOOP, UP, DEPOSIT, IN
    }

    public enum Mode {
        LOW, HIGH, MED
    }

    // PORTME Use REV PID
    private static final double HOLD_LIFT_POWER = 0.15;
    private int currentPosition;
    // PORTME
    private static final int lowPosition = 50;
    private static final int medPosition = 555;
    private static final int highPosition = 1400;

    private final DcMotor liftMotor1;
    private final DcMotor liftMotor2;
    private final DcMotor liftMotor3;
    private final DcMotor liftMotor4;
    // private final RevMagnetSensor liftLimitSwitch;

    //TODO: adjust these values maybe
    private TimerCanceller resetLiftTimerCanceller = new TimerCanceller(750);
    private TimerCanceller extendLiftTimerCanceller = new TimerCanceller(500);

    private Goal goal = Goal.IN;
    private Mode mode = Mode.LOW;

    public Lift(HardwareMap map) {
        liftMotor1 = map.dcMotor.get("liftMotor1");
        liftMotor2 = map.dcMotor.get("liftMotor2");
        liftMotor3 = map.dcMotor.get("liftMotor3");
        liftMotor4 = map.dcMotor.get("liftMotor4");
        // liftLimitSwitch = new RevMagnetSensor(map.get(DigitalChannel.class, "liftLimitSwitch"));

        // PORTME
        // liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void initAuto() {
       runLiftToPosition(Mode.LOW);
    }
    public void initBlockAuto() {

    }

    @Override
    public void initTeleOp() {
       runLiftToPosition(Mode.LOW);
    }

    @Override
    public void update() {
        switch (goal) {
            case OPEN_LOOP:
                break;
            case DEPOSIT:
                break;
            case UP:
                break;
            case IN:
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

    public void setLiftPower(double power) {
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
        liftMotor3.setPower(power);
        liftMotor4.setPower(power);
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
    public double getLiftEncoderTicks() {return liftMotor1.getCurrentPosition();}

    // PORTME
    public void runLiftToPosition(Mode mode) {
        switch (mode) {
            case LOW:
                // liftMotor.setTargetPosition(lowPosition);
                // liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // liftMotor.setPower(-0.7);
                // if (lowPosition == liftMotor.getCurrentPosition()) {
                    // liftMotor.setPower(0);
                // }
                break;
            case MED:
                // liftMotor.setTargetPosition(medPosition);
                // liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // liftMotor.setPower(0.7);
                // if (medPosition == liftMotor.getCurrentPosition()) {
                    // holdOrStopLift();
                // }
                break;
            case HIGH:
                // liftMotor.setTargetPosition(highPosition);
                // liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // liftMotor.setPower(0.7);
                // if (highPosition == liftMotor.getCurrentPosition()) {
                    // holdOrStopLift();
                //}
                break;
        }
    }
}
