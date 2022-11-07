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
    //private final ServoImplEx rightFlipServo;
    private final ServoImplEx leftFlipServo;
    // Alternate timed servo code
    // private final TimedServo timedRightFlipServo;
    private final TimedServo timedLeftFlipServo;
    private FlipPosition curFlipPosition = FlipPosition.INIT;
    private ClawPosition curClawPosition = ClawPosition.INIT;

    private final double maxLeft=1.0;
    private final double midLeft= 0.6;
    private final double minLeft=0.197;
    // private final double midRight= 0;
    // private final double midRight= 0.6;
    // private final double minRight=0.0245;

    private Goal currentGoal;

    private TimerCanceller flipUpCanceller = new TimerCanceller(500);
    //private TimerCanceller releaseCanceller = new TimerCanceller(1000);
    //private TimerCanceller flipBackCanceller = new TimerCanceller(2000);
    private TimerCanceller clawOpenCanceller = new TimerCanceller(250);
    private TimerCanceller depositorStateCanceller = new TimerCanceller(2000);

    public Claw(HardwareMap map) {
        clawServoRight = map.get(ServoImplEx.class,"clawServoRight");
        clawServoLeft = map.get(ServoImplEx.class,"clawServoLeft");
        leftFlipServo = map.get(ServoImplEx.class,"leftFlipServo");
        //rightFlipServo = map.get(ServoImplEx.class,"RightFlipServo");
        // Alternate timed servo code
        timedLeftFlipServo = new TimedServo(leftFlipServo);
        // timedRightFlipServo = new TimedServo(rightFlipServo);

        currentGoal = Goal.OPEN_LOOP;
    }

    @Override
    public void initAuto() {
    }

    /*
    @Override
    public void initAuto()
    {
        NanoClock clock = NanoClock.system();
        double now;

        leftFlipServo.setPosition(minLeft);
        curFlipPosition = FlipPosition.COLLECT;
        now = clock.seconds();
        while (clock.seconds() < (now + 0.5 ) ) { }
        setClawServoPosition(ClawPosition.CLOSED);
    }
    */

    @Override
    public void initTeleOp() {
        setClawServoPosition(ClawPosition.OPEN);
        leftFlipServo.setPosition(minLeft);
        curFlipPosition = FlipPosition.COLLECT;
    }

    @Override
    public void updateComponent() {
        // Alternate with TimedServo
        //timedRightFlipServo.update();
        timedLeftFlipServo.update();
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
                setClawServoPosition(ClawPosition.RELEASE);
                break;
            case RETURN_MID:
                setFlipServoPosition(FlipPosition.MID);
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
            switch (currentGoal) {
                case OPEN_LOOP:
                    depositorStateCanceller.reset(1);
                    break;
                case COLLECT_MID:
                    depositorStateCanceller.reset(1000);
                    break;
                case FLIP:
                    depositorStateCanceller.reset(600);
                    break;
                case RELEASE:
                    depositorStateCanceller.reset(250);
                    break;
                case RETURN_MID:
                    depositorStateCanceller.reset(600);
                    break;
                case RESET:
                    depositorStateCanceller.reset(600);
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

    public void setClawServoWordPosition(double position) {
        clawServoRight.setPosition(position);
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
                    timedLeftFlipServo.setTimedPosition(minLeft, 500);
                } else if( curFlipPosition == FlipPosition.DEPOSIT) {
                    timedLeftFlipServo.setTimedPosition(minLeft, 1000);
                }
                break;
            case MID:
                if( curFlipPosition != FlipPosition.MID) {
                    timedLeftFlipServo.setTimedPosition(midLeft, 500);
                }
                break;
            case DEPOSIT:
                if( curFlipPosition == FlipPosition.MID) {
                    timedLeftFlipServo.setTimedPosition(maxLeft, 500);
                } else if( curFlipPosition == FlipPosition.COLLECT) {
                    timedLeftFlipServo.setTimedPosition(maxLeft, 1000);
                }
                break;
        }
        curFlipPosition = position;
    }

}