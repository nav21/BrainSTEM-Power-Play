package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.util.NanoClock;


import org.firstinspires.ftc.teamcode.autonomous.cancellers.TimerCanceller;
import org.firstinspires.ftc.teamcode.autonomous.enums.ClawPosition;
import org.firstinspires.ftc.teamcode.autonomous.enums.FlipPosition;
import org.firstinspires.ftc.teamcode.utils.Component;

public class Claw implements Component {
    public enum Goal {
        OPEN_LOOP,  COLLECT, TRANSFER
    }

    private final ServoImplEx clawServo;
    private final ServoImplEx rightFlipServo;
    private final ServoImplEx leftFlipServo;

    private Goal currentGoal;

    private TimerCanceller clawStopCanceller = new TimerCanceller(500);

    public Claw(HardwareMap map) {
        clawServo = map.get(ServoImplEx.class,"clawServo");
        leftFlipServo = map.get(ServoImplEx.class,"leftFlipServo");
        rightFlipServo = map.get(ServoImplEx.class,"RightFlipServo");

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
        switch (currentGoal) {
            case OPEN_LOOP:
                break;
            case COLLECT:
                break;
            case TRANSFER:
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

            clawStopCanceller.reset();
        }
    }

    public Goal getCurrentGoal() {
        return currentGoal;
    }

    public void setClawServoWordPosition(double position) {
        clawServo.setPosition(position);
    }
    //TODO: FIX THESE VALUES
    public void setClawServoPosition(ClawPosition position) {
        switch (position) {
            case OPEN:
                clawServo.setPosition(0.2);
                break;
            case CLOSED:
                clawServo.setPosition(0.9);
                break;
        }
    }
// right collect 0.83 left collect 0.197 right deposit 0.0245 left deposit 1
    public void setFlipServoPosition(FlipPosition position) {
        double maxRight=0.83;
        double maxLeft=1.0;
        double minRight=0.0245;
        double minLeft=0.197;
        double deltaRight = maxRight-minRight;
        double deltaLeft = maxLeft-minLeft;
        NanoClock clock = NanoClock.system();
        double tgt = 0;
        switch (position) {
            case COLLECT:
                for (double i =0; i <= 1.0; i += 0.01) {
                    rightFlipServo.setPosition(minRight+(deltaRight*i));
                    leftFlipServo.setPosition(maxLeft-(deltaLeft*i));
                    tgt = clock.seconds()+0.020;
                    while (clock.seconds() < tgt) { }
                }
                break;
            case DEPOSIT:
                for (double i =0; i <= 1.0; i += 0.01) {
                    rightFlipServo.setPosition(maxRight-(deltaRight*i));
                    leftFlipServo.setPosition(minLeft+(deltaLeft*i));
                    tgt = clock.seconds()+0.020;
                    while (clock.seconds() < tgt) { }
                }
                break;
        }
    }
}
