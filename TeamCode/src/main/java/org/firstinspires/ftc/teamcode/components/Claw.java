package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.autonomous.cancellers.TimerCanceller;
import org.firstinspires.ftc.teamcode.autonomous.enums.ClawPosition;
import org.firstinspires.ftc.teamcode.utils.Component;

public class Claw implements Component {
    public enum Goal {
        OPEN_LOOP,  COLLECT, TRANSFER
    }

    private final ServoImplEx clawServo;
    private Goal currentGoal;

    private TimerCanceller clawStopCanceller = new TimerCanceller(500);

    public Claw(HardwareMap map) {
        clawServo = map.get(ServoImplEx.class,"clawServo");

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
                clawServo.setPosition(1);
                break;
            case CLOSED:
                clawServo.setPosition(0.7);
                break;
        }
    }
}
