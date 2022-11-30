package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.autonomous.cancellers.TimerCanceller;
import org.firstinspires.ftc.teamcode.autonomous.enums.LiftPosition;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.utils.Component;
import org.firstinspires.ftc.teamcode.utils.MiniPID;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by parvs on 11/9/2018.
 */

public class Lift implements Component {

    private double U = 1.0 / 1500.0;
    private double Period = 4.5;
    private double P = U * 0.45 ;               // proportional scaling
    private double I = (0.5 * P) / Period;    // integral scaling
    private double D = 0;                     // derivative scaling
    //private double P = U * 0.45;               // proportional scaling
    //private double I = 0;
    //private double D = 0;                     // derivative scaling

    //private double F = 0.3/34000;    // @ 13V resting (12.8V under load)
    private double F = 0;    // @ 13V resting (12.8V under load)
    private double vF = F;            // Voltage adjusted F (use battery reading to help here)
    private static double MAX_LIFT_UP_PWR = 0.95 ;
    public static double MIN_LIFT_UP_PWR = 0.01 ;

    private NanoClock PIDClock = NanoClock.system();   // Keep time interval because we can't call at a regular interval

    public double tgtPos = -1.0 ;                     // Target
    public double pidTgtPos = -1.0 ;                     // Target
    public double PIDTime = 0.025;                     // Ideal/minimum interval
    private double PIDStartTime = PIDClock.seconds();  // When did we start
    private double nextPID = PIDStartTime;             // When is the next time we can process PID
    public int PIDCount = 0;                          // Just tracking our first call to know when to use FeedF
    public int PIDSkipCount = 0;
    private double lastPIDTime = PIDTime;              // Last time through the update code
    private double prevPIDTime = PIDTime;              // Previous time actually updated

    // Create and init our PID
    public MiniPID pid = new MiniPID(P,I,D,F);                // Create the PID

    public enum Goal {
        OPEN_LOOP, UP, DOWN, MANUAL
    }

    public enum Mode {
        HIGH, MED, LOW, JUNC, REST, CONE_5, CONE_4, CONE_3, CONE_2, INIT
    }

    //private static final double HOLD_LIFT_POWER = 0.15;
    //private int currentPosition;
    //private final double heightScale = 38.0/61.0;  // 1/1.6053
    private final int restPosition = (int)(240);
    private final int junctionPosition = (int)(3210);
    private final int lowPosition = (int)(25684);
    private final int medPosition = (int)(43545);
    private final int highPosition = (int)(61000);

    private final int cone5 = 20+(2025*4); // 8120
    private final int cone4 = 20+(2025*3); // 6095
    private final int cone3 = 20+(2025*2); // 4070
    private final int cone2 = 20+(2025*1); // 2045

    private NanoClock DownClock = NanoClock.system();   // Keep time interval because we can't call at a regular interval
    private double prevDownTime = 0.0;
    private double downRate = (1605.0/0.020);  // 1605 ticks / 20 ms
    private LiftPosition downPosition = LiftPosition.REST ;

    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;

    //TODO: adjust these values maybe
    //private static TimerCanceller resetLiftTimerCanceller = new TimerCanceller(750);
    //private static TimerCanceller extendLiftTimerCanceller = new TimerCanceller(500);

    private Goal goal = Goal.OPEN_LOOP;
    private Goal prevGoal = Goal.OPEN_LOOP;
    private Mode mode = Mode.HIGH;
    public double pwr=0.0;
    private boolean autoStackMode=false;

    public Lift(HardwareMap map) {
        fl = map.dcMotor.get("flandperpendicularEncoder");
        fr = map.dcMotor.get("frandparallelEncoder");
        bl = map.dcMotor.get("blandliftEncoder");
        br = map.dcMotor.get("br");

        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pid.reset();                                       // Out PID keeps state, clear it all out
        pid.setOutputLimits(0.0, MAX_LIFT_UP_PWR);          // Make sure we don't exceed some maximum rating
        vF = F;
        pid.setPID(P, I, D, vF);                              // Set out params
    }

    public void zeroLift() {
        Goal goal = getGoal();

        setGoal(Goal.MANUAL);
        setMotorPowers(-0.4);

        NanoClock clock = NanoClock.system();
        double now;

        now = clock.seconds();
        while (clock.seconds() < (now + 1) ) { }

        setMotorPowers(0.0);

        now = clock.seconds();
        while (clock.seconds() < (now + 0.500) ) { }

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setGoal(goal);
    }

    private LiftPosition getNextDown( LiftPosition current ) {
        LiftPosition next = current ;
        switch(current) {
            case CONE_5 :
                next = LiftPosition.CONE_4;
                break;
            case CONE_4 :
                next = LiftPosition.CONE_3;
                break;
            case CONE_3 :
                next = LiftPosition.CONE_2;
                break;
            case CONE_2 :
                next = LiftPosition.REST;
                break;
            case REST :
                next = LiftPosition.REST;
                break;
        }
        return (next) ;
    }

    public void autoStackModeButtonPress() {
        if (autoStackMode) {
            // If we press button in auto mode
            if (goal.equals(Goal.DOWN)) {
                // Pressed while we are DOWN, means advance one position
                if(downPosition == LiftPosition.REST) {
                    // If we are at the bottom, then we should just leave auto mode
                    autoStackMode = false;
                    runLiftToPosition(downPosition);
                } else {
                    // We set the position manually because we're not leaving 'DOWN' goal
                    runLiftToPosition(downPosition);
                    downPosition = getNextDown(downPosition);
                }
            } else {
                // Pressed while we're 'UP' -- ignored
                return;
            }
        } else {
            // If we are not in auto mode, then go to CONE_5 position and Auto mode
            downPosition = LiftPosition.CONE_5;
            autoStackMode = true;

            if(goal.equals(Goal.DOWN)) {
                // If DOWN, then move us manually and then advance
                runLiftToPosition(downPosition);
                downPosition = getNextDown(downPosition);
            } else {
                // If we were 'UP', switch to DOWN and update routine will move us
                setGoal(Goal.DOWN);
            }
        }
    }

    @Override
    public void initAuto() {
        zeroLift();
    }

    @Override
    public void initTeleOp() {
        // zeroLift();
    }

    @Override
    public void updateComponent() {
        double now = DownClock.seconds();
        double curPos = getLiftEncoderTicks();
        double downAmount = 0.0 ;

        if(tgtPos >= curPos) {
            pidTgtPos = tgtPos;
            pid.setOutputLimits(MIN_LIFT_UP_PWR, MAX_LIFT_UP_PWR);
        } else {
            // Target a down amount of 1000 ticks per 30ms
            downAmount = (now-prevDownTime) * downRate;
            pidTgtPos = Math.max(tgtPos, curPos - downAmount);
            if (curPos < 28000) {
                pid.setOutputLimits(-0.45, MAX_LIFT_UP_PWR);
            } else if (curPos < 55000) {
                pid.setOutputLimits(-0.04, MAX_LIFT_UP_PWR);
            } else {
                pid.setOutputLimits(0.0, MAX_LIFT_UP_PWR);
            }
        }
        updateLiftPID();
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
                }else if (mode.equals(Mode.CONE_5)) {
                    runLiftToPosition(LiftPosition.CONE_5);
                }else if (mode.equals(Mode.CONE_4)) {
                    runLiftToPosition(LiftPosition.CONE_4);
                }else if (mode.equals(Mode.CONE_3)) {
                    runLiftToPosition(LiftPosition.CONE_3);
                }else if (mode.equals(Mode.CONE_2)) {
                    runLiftToPosition(LiftPosition.CONE_2);
                }
                break;
            case DOWN:
                if (prevGoal != Goal.DOWN) {
                    runLiftToPosition(downPosition);
                    if (autoStackMode) {
                        if (downPosition == LiftPosition.REST) {
                            autoStackMode = false;
                        }
                        downPosition = getNextDown(downPosition);
                    }
                }
                break;
        }
        prevGoal = goal;
    }

    public void setGoal(Goal goal) {
        if (this.goal != goal) {
            this.goal = goal;
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

    public LiftPosition getDownPosition() {
        return downPosition;
    }

    public String test() {
        String failures = "";

        return failures;
    }

    public void setMotorPowers(double pwr){
        setMotorPowers(pwr,pwr,pwr,pwr);
    }

    public void setMotorPowers(double vFL, double vFR, double vBL, double vBR) {
        fl.setPower(vFL);
        fr.setPower(vFR);
        bl.setPower(vBL);
        br.setPower(vBR);
    }

    public double getLiftEncoderTicks() {

        return bl.getCurrentPosition();

    }
    public double getEncoderTicks() {

        return fl.getCurrentPosition();

    }
    public double get2EncoderTicks() {

        return fr.getCurrentPosition();

    }

    public void runLiftToPosition(LiftPosition position) {
        switch (position) {
            case REST:
                setLiftPos(restPosition);
                break;
            case JUNC:
                setLiftPos(junctionPosition);
                break;
            case LOW:
                setLiftPos(lowPosition);
                break;
            case MED:
                setLiftPos(medPosition);
                break;
            case HIGH:
                setLiftPos(highPosition);
                break;
            case CONE_4:
                setLiftPos(cone4);
                break;
            case CONE_5:
                setLiftPos(cone5);
                break;
            case CONE_3:
                setLiftPos(cone3);
                break;
            case CONE_2:
                setLiftPos(cone2);
                break;
        }
    }

    public double getTgtPos(){
        return(tgtPos);
    }

    public void setLiftPos(double tgtPos) {

        double curPos = getLiftEncoderTicks();

        if (tgtPos != this.tgtPos) {
            this.tgtPos = tgtPos;
            // Set new target and reset PID variables
            PIDCount = 0;
            PIDSkipCount = 0;
            PIDStartTime = PIDClock.seconds();
            nextPID = PIDStartTime;
            pid.reset();

            if(getGoal() == Goal.UP ) {
                pid.setOutputLimits(MIN_LIFT_UP_PWR, MAX_LIFT_UP_PWR);
            } else {
                // These controls
                pid.setOutputLimits(-0.25, 0.2);             // Make sure we don't exceed some maximum rating
            }

            // Force an update, the first update only sets Feed-forward
            updateLiftPID();
        }
    }

    public void updateLiftPID() {
        double now = PIDClock.seconds() ;
        double curPos;

        // If enough time has passed
        if ( nextPID < now ) {
            // Get out if we are in manual control
            if(goal == Goal.MANUAL){
                return;
            }

            // And we are supposed to be powered
            // Make a different way to request lift stop
            if (pidTgtPos >= 0.0) {

                PIDCount++;

                // Get our current position (average? max? single?)
                curPos = getLiftEncoderTicks();

                // Need 2 samples to do anything that has time weighting
                if (PIDCount > 1) {

                    // Compute PID values and include some time skew if there was any
                    pwr = pid.getOutput(curPos, pidTgtPos, ((now-prevPIDTime)/PIDTime));

                } else {
                    // just set a fake feed-forward value on the first sample.
                    pwr = (vF * (pidTgtPos-curPos));
                }

                // Set the power
                pwr = Range.clip(pwr, -MAX_LIFT_UP_PWR, MAX_LIFT_UP_PWR);
                setMotorPowers(pwr);

                prevPIDTime = now;

                // 'Schedule' the next PID check
                nextPID = now + PIDTime;
                lastPIDTime = now;
            } else {
                setMotorPowers(0.0);
                // 'Schedule' the next PID check
                // Maybe we should just be turning the PID off now?
                nextPID = now + PIDTime;
                lastPIDTime = now;
            }
        } else {
            PIDSkipCount++;
        }
    }
}
