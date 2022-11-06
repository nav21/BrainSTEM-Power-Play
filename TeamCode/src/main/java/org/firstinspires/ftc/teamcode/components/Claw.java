package org.firstinspires.ftc.teamcode.components;

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
        OPEN_LOOP,  COLLECT, RETURN, DEPOSIT
    }

    private final ServoImplEx clawServoRight;
    private final ServoImplEx clawServoLeft;
    private final ServoImplEx rightFlipServo;
    private final ServoImplEx leftFlipServo;
    // Alternate timed servo code
    // private final TimedServo timedRightFlipServo;
    private final TimedServo timedLeftFlipServo;


    private Goal currentGoal;

    private TimerCanceller flipUpCanceller = new TimerCanceller(1000);
    private TimerCanceller releaseCanceller = new TimerCanceller(1000);
    private TimerCanceller flipBackCanceller = new TimerCanceller(2000);
    private TimerCanceller clawGrabCanceller = new TimerCanceller(4000);



    public Claw(HardwareMap map) {
        clawServoRight = map.get(ServoImplEx.class,"clawServoRight");
        clawServoLeft = map.get(ServoImplEx.class,"clawServoLeft");
        leftFlipServo = map.get(ServoImplEx.class,"leftFlipServo");
        rightFlipServo = map.get(ServoImplEx.class,"RightFlipServo");
        // Alternate timed servo code
        timedLeftFlipServo = new TimedServo(leftFlipServo);
        // timedRightFlipServo = new TimedServo(rightFlipServo);

        currentGoal = Goal.OPEN_LOOP;
    }

    @Override
    public void initBlockAuto() {
        setClawServoPosition(ClawPosition.OPEN);
    }
    public void initAuto() {
        setClawServoPosition(ClawPosition.OPEN);
    }

    @Override
    public void initTeleOp() {
        setClawServoPosition(ClawPosition.OPEN);
    }


    @Override
    public void update() {
        // Alternate with TimedServo
        //timedRightFlipServo.update();
        timedLeftFlipServo.update();
        switch (currentGoal) {
            case OPEN_LOOP:
                break;
            case COLLECT:
                if(flipUpCanceller.isConditionMet()) {
                  setFlipServoPosition(FlipPosition.MID);
                }
                setClawServoPosition(ClawPosition.OPEN);
            case DEPOSIT:
                if (clawGrabCanceller.isConditionMet()){
                    setClawServoPosition(ClawPosition.OPEN);
                }
                if (flipBackCanceller.isConditionMet())
                    setFlipServoPosition(FlipPosition.COLLECT);

                if (releaseCanceller.isConditionMet()) {
                    setClawServoPosition(ClawPosition.RELEASE);
                }
                setFlipServoPosition(FlipPosition.DEPOSIT);

                break;
            case RETURN:
                break;
        }
    }

    public String test() {
        String failures = "";

        return failures;
    }

    public void setCurrentGoal(Goal currentGoal) {
        if (this.currentGoal != currentGoal) {
            this.currentGoal = currentGoal;

            flipUpCanceller.reset();
            flipBackCanceller.reset();
            releaseCanceller.reset();
            clawGrabCanceller.reset();
        }
    }

    public Goal getCurrentGoal() {
        return currentGoal;
    }

    public void setClawServoWordPosition(double position) {
        clawServoRight.setPosition(position);
    }
    //TODO: FIX THESE VALUES
    public void setClawServoPosition(ClawPosition position) {
        switch (position) {
            case CLOSED:
                clawServoRight.setPosition(0.17);
                clawServoLeft.setPosition(0.83);
                break;
            case OPEN:
                clawServoRight.setPosition(0.35);
                clawServoLeft.setPosition(0.65);
                break;
            case RELEASE:
                clawServoRight.setPosition(0.2);
                clawServoLeft.setPosition(0.8);
                break;
        }
    }
    // right collect 0.83 left collect 0.197 right deposit 0.0245 left deposit 1
    public void setFlipServoPosition(FlipPosition position) {
      //  double maxRight=0.83;
        double maxLeft=1.0;
        double midRight= 0; //TODO: CHANGE MID VALUES
        double midLeft= 0.5;
       // double minRight=0.0245;
        double minLeft=0.207;
      //  double deltaRight = maxRight-minRight;
        double deltaLeft = maxLeft-minLeft;
        //double deltaMidCollectRight = maxRight-midRight;
        double deltaMidCollectLeft = maxLeft-midLeft;
       // double deltaMidDepositRight = midRight-minRight;
        double deltaMidDepositLeft = midLeft-minLeft;
        NanoClock clock = NanoClock.system();
        double tgt = 0;
        switch (position) {
            case COLLECT:
                // The 'old way'
              //  rightFlipServo.setPosition(maxRight);
               // leftFlipServo.setPosition(minLeft);
                // Alternate with TimedServo (enable update above)
                // timedRightFlipServo.setPosition(maxRight, 1.5);
                timedLeftFlipServo.setTimedPosition(minLeft, 1500);
//                for (double i =0; i <= 1.0; i += 0.01) {
//                    rightFlipServo.setPosition(maxRight-(deltaRight*i));
//                    leftFlipServo.setPosition(minLeft+(deltaLeft*i));
//                    tgt = clock.seconds()+0.020;
//                    while (clock.seconds() < tgt) { }
//                }
                break;
            case MID:
                // The 'old way'
                //rightFlipServo.setPosition(minRight);
              //  leftFlipServo.setPosition(maxLeft);
                // Alternate with TimedServo (enable update above)
                // timedRightFlipServo.setPosition(minRight, 1.5);
                 timedLeftFlipServo.setTimedPosition(midLeft, 1.5);
//                for (double i =0; i <= 1.0; i += 0.01) {
//                    rightFlipServo.setPosition(minRight+(deltaMidDepositRight*i));
//                    leftFlipServo.setPosition(maxLeft-(deltaMidDepositLeft*i));
//                    tgt = clock.seconds()+0.020;
//                    while (clock.seconds() < tgt) { }
//                }
                break;
            case DEPOSIT:
                // The 'old way'
//                rightFlipServo.setPosition(minRight);
//                leftFlipServo.setPosition(maxLeft);
                // Alternate with TimedServo (enable update above)
                // timedRightFlipServo.setPosition(minRight, 1.5);
                 timedLeftFlipServo.setTimedPosition(maxLeft, 1.5);
//                for (double i =0; i <= 1.0; i += 0.01) {
//                    rightFlipServo.setPosition(maxRight-(deltaMidCollectRight*i));
//                    leftFlipServo.setPosition(minLeft+(deltaMidDepositLeft*i));
//                    tgt = clock.seconds()+0.020;
//                    while (clock.seconds() < tgt) { }
//                }
                break;
        }
    }

}