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
    private double MAX_LIFT_UP_PWR = 0.95 ;
    public double MIN_LIFT_UP_PWR = 0.01 ;

    private NanoClock PIDClock = NanoClock.system();   // Keep time interval because we can't call at a regular interval
    private NanoClock DownClock = NanoClock.system();   // Keep time interval because we can't call at a regular interval
    private double nextDown = 0.0;
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
        OPEN_LOOP, UP, DOWN
    }

    public enum Mode {
        LOW, HIGH, MED, JUNC, INIT, CONE_5, CONE_4, CONE_3, CONE_2,
    }

    //private static final double HOLD_LIFT_POWER = 0.15;
    private int currentPosition;
    private final double heightScale = 38.0/61.0;
    private final int restPosition = (int)(200.0 / heightScale);
    private final int junctionPosition = (int)(2000.0 / heightScale);
    private final int lowPosition = (int)((16000.0 / heightScale)+1000);
    private final int medPosition = (int)((27500.0 / heightScale)+400);
    private final int highPosition = (int)((38000.0 / heightScale)+1000);

    private final int cone5 = restPosition+(2025*4);
    private final int cone4 = restPosition+(2025*3);
    private final int cone3 = restPosition+(2025*2);
    private final int cone2 = restPosition+(2025*1);

    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;

    //TODO: adjust these values maybe
    private TimerCanceller resetLiftTimerCanceller = new TimerCanceller(750);
    private TimerCanceller extendLiftTimerCanceller = new TimerCanceller(500);

    private Goal goal = Goal.OPEN_LOOP;
    private Goal prevGoal = Goal.OPEN_LOOP;
    private Mode mode = Mode.HIGH;
    public double pwr=0.0;

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
        pid.setOutputLimits(0.0,MAX_LIFT_UP_PWR);          // Make sure we don't exceed some maximum rating
        vF = F;
        pid.setPID(P,I,D,vF);                              // Set out params
    }

    @Override
    public void initAuto() {
    }

    @Override
    public void initTeleOp() {
    }

    @Override
    public void updateComponent() {
        double now = DownClock.seconds();
        double curPos = getLiftEncoderTicks();

        if(tgtPos >= curPos) {
            pidTgtPos = tgtPos;
            pid.setOutputLimits(MIN_LIFT_UP_PWR, MAX_LIFT_UP_PWR);
        } else {
            if (nextDown < now) {
                pidTgtPos = Math.max(tgtPos, curPos - (1000.0 / heightScale));
                nextDown = now + 0.03;
                if (curPos < 25000) {
                    pid.setOutputLimits(-0.45, MAX_LIFT_UP_PWR);
                } else if (curPos < 55000) {
                    pid.setOutputLimits(-0.04, MAX_LIFT_UP_PWR);
                } else {
                    pid.setOutputLimits(0.0, MAX_LIFT_UP_PWR);
                }
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
                    setLiftPos(restPosition);
                }
                break;
        }
        prevGoal = goal;
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

            if( getGoal() == Goal.UP ) {
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
        //double pwr;

        // If enough time has passed
        if ( nextPID < now ) {

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
                setMotorPowers(pwr,pwr,pwr,pwr);

                prevPIDTime = now;

                // 'Schedule' the next PID check
                nextPID = now + PIDTime;
                lastPIDTime = now;
            } else {
                setMotorPowers(0.0,0.0,0.0,0.0);
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
