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
        OPEN_LOOP,  COLLECT_MID, RETURN_MID, RESET, RELEASE, FLIP, COLLECT, DEPOSIT, DEPOSIT_2, DEPOSIT_3,DEPOSIT_4,DEPOSIT_5, DEPOSIT_6, COLLECT_25, RETURN_SPECIAL
    }

    private final ServoImplEx clawServoRight;
    private final ServoImplEx clawServoLeft;
    private final ServoImplEx rightFlipServo;
    private final ServoImplEx leftFlipServo;
    private final TimedServo timedRightFlipServo;
    private final TimedServo timedLeftFlipServo;
    private FlipPosition curFlipPosition = FlipPosition.UNKNOWN;
    private ClawPosition curClawPosition = ClawPosition.UNKNOWN;

    private final double depositLeft=0.98; // 1
    private final double midLeft= 0.6;
    private final double initLeft=0.4;
    private final double collectLeft=0.203;//0.197
    private final double partwayLeft= 0.4;
    private final double partwayRight = 1.0 - partwayLeft;
    private final double depositRight= 1.0 - depositLeft;
    private final double midRight= 1.0 - midLeft;
    private final double initRight= 1.0 - initLeft;
    private final double collectRight= 1.0 - collectLeft;

    private Goal currentGoal;
    private boolean lastGoalCollect25 = false;

    private TimerCanceller closeV4BCanceller = new TimerCanceller(1);
    private TimerCanceller flipUpCanceller = new TimerCanceller(300);
    private TimerCanceller flipUpCanceller2 = new TimerCanceller(300);
    private TimerCanceller clawOpenCanceller = new TimerCanceller(200);
    private TimerCanceller openClawCanceller = new TimerCanceller(1000);
    private TimerCanceller depositorStateCanceller = new TimerCanceller(2000);
    private TimerCanceller returnMidFlipCanceller = new TimerCanceller(250);
    private TimerCanceller autoFlipUpCanceller = new TimerCanceller(400);
    private TimerCanceller autoReleaseCanceller = new TimerCanceller(300);
    private TimerCanceller disableServoCanceller = new TimerCanceller((700*(1.0/3.0)));
    private TimerCanceller disableServoCanceller2 = new TimerCanceller(700);
    private TimerCanceller dropDelayCanceller = new TimerCanceller(350);
    private TimerCanceller closeDelayCanceller = new TimerCanceller(200);
    private TimerCanceller openClawCanceller2 = new TimerCanceller(700*(2.0/3.0));
    private TimerCanceller specialDisableFlipCanceller = new TimerCanceller(210);
    private TimerCanceller funnyDisableCollectFlipCanceller = new TimerCanceller(1150);







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
            //TODO NEW CODE
            case COLLECT_25:
                if(autoFlipUpCanceller.isConditionMet()) {
                    enableFlipServo();
                    setFlipServoPosition(FlipPosition.PARTWAY25);

                }
                setClawServoPosition(ClawPosition.CLOSED);
                lastGoalCollect25 = true;
                break;
            case DEPOSIT:
                if (lastGoalCollect25) {
                    if(specialDisableFlipCanceller.isConditionMet()) {
                        disableFlipServo();
                    }
                    setClawServoPosition(ClawPosition.OPEN);
                    setFlipServoPosition(FlipPosition.COLLECT);
                } else {
                    setClawServoPosition(ClawPosition.OPEN);
                    dropDelayCanceller.reset();
                    setCurrentGoal(Goal.DEPOSIT_2);
                }
                break;
            case DEPOSIT_2:
                if(dropDelayCanceller.isConditionMet()){
                    setClawServoPosition(ClawPosition.CLOSED);
                    closeDelayCanceller.reset();
                    setCurrentGoal(Goal.DEPOSIT_3);
                }
                break;
            case DEPOSIT_3:
                if(closeDelayCanceller.isConditionMet()){
                    enableFlipServo();
                    openClawCanceller2.reset();
                    setFlipServoPosition(FlipPosition.COLLECT);
                    setCurrentGoal(Goal.DEPOSIT_4);
                }
                break;
            case DEPOSIT_4:
                if(openClawCanceller2.isConditionMet()){
                    setClawServoPosition(ClawPosition.OPEN);
                    disableServoCanceller.reset();
                    setCurrentGoal(Goal.DEPOSIT_5);
                }
                break;
            case DEPOSIT_5:
                if(disableServoCanceller.isConditionMet()){
                    disableFlipServo();
                    setCurrentGoal(Goal.DEPOSIT_6);
                }
                break;
            case DEPOSIT_6:
                break;
            case COLLECT:
                if(disableServoCanceller2.isConditionMet()) {
                    disableFlipServo();
                }
                setFlipServoPosition(FlipPosition.DEPOSIT);
                lastGoalCollect25 = false;
                break;
            case RETURN_SPECIAL:
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
            autoFlipUpCanceller.reset();
            autoReleaseCanceller.reset();
            flipUpCanceller.reset();
            flipUpCanceller2.reset();
            clawOpenCanceller.reset();
            openClawCanceller.reset();
            returnMidFlipCanceller.reset();
            disableServoCanceller.reset();
            disableServoCanceller2.reset();
            specialDisableFlipCanceller.reset();
            funnyDisableCollectFlipCanceller.reset();
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
                case COLLECT_25:
                    depositorStateCanceller.reset(250);
                    break;
                case DEPOSIT:
                    depositorStateCanceller.reset(250);
                    break;
                case COLLECT:
                    depositorStateCanceller.reset(250);
                    break;
                default:
                    depositorStateCanceller.reset(500);
                    break;
            }
        }
    }

    public Goal getCurrentGoal() {
        return currentGoal;
    }

    public boolean getPrevGoalCollect25() {
        return lastGoalCollect25;
    }


    public double getRightClawPosition() {
        return(clawServoRight.getPosition());
    }

    public double getLeftClawPosition() {
        return(clawServoLeft.getPosition());
    }
    public void setRightClawPosition(double position) {
        clawServoRight.setPosition(position);
    }

    public void setLeftClawPosition(double position) {
        clawServoLeft.setPosition(position);
    }

    public void setLeftFlipPosition(double position) {
        leftFlipServo.setPosition(position);
    }
    public void setRightFlipPosition(double position) {
        rightFlipServo.setPosition(position);
    }

    public void disableFlipServo() {
        rightFlipServo.setPwmDisable();
        leftFlipServo.setPwmDisable();
    }
    public void enableFlipServo() {
        rightFlipServo.setPwmEnable();
        leftFlipServo.setPwmEnable();
    }

    //TODO: FIX THESE VALUES
    public void setClawServoPosition(ClawPosition position) {
        if (curClawPosition != position) {
            switch (position) {
                case CLOSED:
                  //  clawServoRight.setPosition(0.15);   // Smaller number is more closed
              //      clawServoLeft.setPosition(0.79);    // Larger number is more closed
                    clawServoRight.setPosition(0.32);   // TORQUE
                    clawServoLeft.setPosition(0.69);    // TORQUE

                    break;
                case RELEASE:
                    clawServoRight.setPosition(0.22);
                    clawServoLeft.setPosition(0.76);
                    break;
                case INIT:
                case OPEN:
                 //   clawServoRight.setPosition(0.30);
               //     clawServoLeft.setPosition(0.66);
                    clawServoRight.setPosition(0.55);   // TORQUE
                    clawServoLeft.setPosition(0.47);    // TORQUE
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
                else if(curFlipPosition == FlipPosition.PARTWAY25) {
                    timedLeftFlipServo.setTimedPosition(collectLeft, 200);
                    timedRightFlipServo.setTimedPosition(collectRight, 200);
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
                else if(curFlipPosition == FlipPosition.PARTWAY25) {
                    timedLeftFlipServo.setTimedPosition(depositLeft, 600);
                    timedRightFlipServo.setTimedPosition(depositRight, 600);
                }
                break;
            case PARTWAY25:
                timedLeftFlipServo.setTimedPosition(partwayLeft, 100);
                timedRightFlipServo.setTimedPosition(partwayRight, 100);
                break;
        }
        curFlipPosition = position;
    }
}
