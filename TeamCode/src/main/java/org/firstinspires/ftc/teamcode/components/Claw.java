package org.firstinspires.ftc.teamcode.components;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.autonomous.cancellers.TimerCanceller;
import org.firstinspires.ftc.teamcode.autonomous.enums.ClawPosition;
import org.firstinspires.ftc.teamcode.autonomous.enums.FlipPosition;
import org.firstinspires.ftc.teamcode.util.TimedServo;
import org.firstinspires.ftc.teamcode.utils.Component;

public class Claw implements Component {
    public enum Goal {
        OPEN_LOOP,  COLLECT_MID, RETURN_MID, RESET, RELEASE, FLIP
    }

    private final ServoImplEx clawServoRight;
    private final ServoImplEx clawServoLeft;
    private final ServoImplEx rightFlipServo;
    private final ServoImplEx leftFlipServo;
    private final TimedServo timedRightFlipServo;
    private final TimedServo timedLeftFlipServo;
    private FlipPosition curFlipPosition = FlipPosition.UNKNOWN;
    private ClawPosition curClawPosition = ClawPosition.UNKNOWN;

    private final double depositLeft=1.0;
    private final double midLeft= 0.6;
    private final double initLeft=0.4;
    private final double collectLeft=0.197;
    private final double depositRight= 1.0 - depositLeft;
    private final double midRight= 1.0 - midLeft;
    private final double initRight= 1.0 - initLeft;
    private final double collectRight= 1.0 - collectLeft;

    private Goal currentGoal;

    private TimerCanceller flipUpCanceller = new TimerCanceller(300);
    private TimerCanceller clawOpenCanceller = new TimerCanceller(200);
    private TimerCanceller depositorStateCanceller = new TimerCanceller(2000);
    private TimerCanceller returnMidFlipCanceller = new TimerCanceller(250);

    public Claw(HardwareMap map) {
        clawServoRight = map.get(ServoImplEx.class, "clawServoRight");
        clawServoLeft = map.get(ServoImplEx.class, "clawServoLeft");
        leftFlipServo = map.get(ServoImplEx.class, "leftFlipServo");
        rightFlipServo = map.get(ServoImplEx.class, "rightFlipServo");
        timedLeftFlipServo = new TimedServo(leftFlipServo);
        timedRightFlipServo = new TimedServo(rightFlipServo);

        currentGoal = Goal.OPEN_LOOP;
    }

    public void initClaw(){
        NanoClock clock = NanoClock.system();
        double now;

        setClawServoPosition(ClawPosition.INIT);
        now = clock.seconds();
        while (clock.seconds() < (now + 0.5) ) { }

        leftFlipServo.setPosition(initLeft);
        rightFlipServo.setPosition(initRight);

        now = clock.seconds();
        while (clock.seconds() < (now + 1) ) { }

        timedLeftFlipServo.setTimedPosition(collectLeft,350);
        timedRightFlipServo.setTimedPosition(collectRight,350);
        curFlipPosition = FlipPosition.COLLECT;

        now = clock.seconds();
        while (clock.seconds() < (now + 0.75) ) {
            timedLeftFlipServo.update();
            timedRightFlipServo.update();
        }
    }

    @Override
    public void initAuto() {
        initClaw();
        setClawServoPosition(ClawPosition.CLOSED);
    }

    @Override
    public void initTeleOp() {
        // initClaw();
        // setClawServoPosition(ClawPosition.OPEN);
    }

    @Override
    public void updateComponent() {
        timedLeftFlipServo.update();
        timedRightFlipServo.update();
        switch (currentGoal) {
            case OPEN_LOOP:
                break;
            case COLLECT_MID:
                if(flipUpCanceller.isConditionMet()) {
                    setFlipServoPosition(FlipPosition.MID);
                }
                setClawServoPosition(ClawPosition.CLOSED);
                break;
            case FLIP:
                setFlipServoPosition(FlipPosition.DEPOSIT);
                break;
            case RELEASE:
                setClawServoPosition(ClawPosition.OPEN);
                break;
            case RETURN_MID:
                if (returnMidFlipCanceller.isConditionMet()) {
                    setFlipServoPosition(FlipPosition.MID);
                }
                setClawServoPosition(ClawPosition.CLOSED);
                break;
            case RESET:
                if (clawOpenCanceller.isConditionMet()) {
                    setClawServoPosition(ClawPosition.OPEN);
                }
                setFlipServoPosition(FlipPosition.COLLECT);
                break;
        }
    }

    public String test() {
        String failures = "";

        return failures;
    }

    public boolean isSafeToChangeClawGoal() {
        return(depositorStateCanceller.isConditionMet());
    }

    public void setCurrentGoal(Goal currentGoal) {
        if (this.currentGoal != currentGoal) {
            this.currentGoal = currentGoal;

            flipUpCanceller.reset();
            clawOpenCanceller.reset();
            returnMidFlipCanceller.reset();
            switch (currentGoal) {
                case OPEN_LOOP:
                    depositorStateCanceller.reset(1);
                    break;
                case COLLECT_MID:
                    depositorStateCanceller.reset(650);
                    break;
                case FLIP:
                    depositorStateCanceller.reset(350);
                    break;
                case RELEASE:
                    depositorStateCanceller.reset(250);
                    break;
                case RETURN_MID:
                    depositorStateCanceller.reset(650);
                    break;
                case RESET:
                    depositorStateCanceller.reset(500);
                    break;
                default:
                    depositorStateCanceller.reset(2000);
                    break;
            }
        }
    }

    public Goal getCurrentGoal() {
        return currentGoal;
    }

    public double getRightClawPosition() {
        return(clawServoRight.getPosition());
    }

    public double getLeftClawPosition() {
        return(clawServoLeft.getPosition());
    }

    public void setLeftFlipPosition(double position) {
        leftFlipServo.setPosition(position);
    }
    public void setRightFlipPosition(double position) {
        rightFlipServo.setPosition(position);
    }

    //TODO: FIX THESE VALUES
    public void setClawServoPosition(ClawPosition position) {
        if (curClawPosition != position) {
            switch (position) {
                case CLOSED:
                    clawServoRight.setPosition(0.15);   // Smaller number is more closed
                    clawServoLeft.setPosition(0.79);    // Larger number is more closed
                    break;
                case RELEASE:
                    clawServoRight.setPosition(0.22);
                    clawServoLeft.setPosition(0.76);
                    break;
                case INIT:
                case OPEN:
                    clawServoRight.setPosition(0.30);
                    clawServoLeft.setPosition(0.66);
                    break;
            }
            curClawPosition = position;
        }
    }
    public void setFlipServoPosition(FlipPosition position) {
        switch (position) {
            case COLLECT:
                if( curFlipPosition == FlipPosition.MID) {
                    timedLeftFlipServo.setTimedPosition(collectLeft, 350);
                    timedRightFlipServo.setTimedPosition(collectRight, 350);
                } else if( curFlipPosition == FlipPosition.DEPOSIT) {
                    timedLeftFlipServo.setTimedPosition(collectLeft, 700);
                    timedRightFlipServo.setTimedPosition(collectRight, 700);
                }
                break;
            case MID:
                if( curFlipPosition != FlipPosition.MID) {
                    timedLeftFlipServo.setTimedPosition(midLeft, 350);
                    timedRightFlipServo.setTimedPosition(midRight, 350);
                }
                break;
            case DEPOSIT:
                if( curFlipPosition == FlipPosition.MID) {
                    timedLeftFlipServo.setTimedPosition(depositLeft, 350);
                    timedRightFlipServo.setTimedPosition(depositRight, 350);
                } else if( curFlipPosition == FlipPosition.COLLECT) {
                    timedLeftFlipServo.setTimedPosition(depositLeft, 700);
                    timedRightFlipServo.setTimedPosition(depositRight, 700);
                }
                break;
        }
        curFlipPosition = position;
    }
}
