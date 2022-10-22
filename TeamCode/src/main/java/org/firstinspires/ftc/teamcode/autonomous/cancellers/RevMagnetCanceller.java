package org.firstinspires.ftc.teamcode.autonomous.cancellers;

import org.firstinspires.ftc.teamcode.wrappers.RevMagnetSensor;

public class RevMagnetCanceller implements Canceller {
    private RevMagnetSensor magnetSensor;

    public RevMagnetCanceller(RevMagnetSensor magnetSensor) {
        this.magnetSensor = magnetSensor;
    }

    @Override
    public boolean isConditionMet() {
        return magnetSensor.isTriggered();
    }
}
