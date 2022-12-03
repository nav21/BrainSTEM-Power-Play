package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.BotLog;
import org.firstinspires.ftc.teamcode.utils.Component;

import java.util.ArrayList;

/**
 * Created by parvs on 7/26/2018.
 */

public class BrainSTEMRobot {
    //Various components of the TeleOp robot
    public BMecanumDrive drive;
    public Claw claw;
    public Lift lift;
    public VoltageSensor Vsense;
    public BotLog logger = new BotLog();

    //List of components to be initialized
    private final ArrayList<Component> components;

    /**
     * Instantiates a new TeleOp robot.
     *
     * @param opMode the op mode
     */
    public BrainSTEMRobot(LinearOpMode opMode) {
        //Get instance of hardware map and telemetry
        HardwareMap map = opMode.hardwareMap;
        logger.LOGLEVEL = logger.LOGDEBUG;

        components = new ArrayList<>();

        //Initialize robot components
        drive = new BMecanumDrive(map);
        lift = new Lift(map);
        claw = new Claw(map);

        for (VoltageSensor sensor : map.voltageSensor) {
            if (Vsense == null) {
                double voltage = sensor.getVoltage();
                if (voltage > 0) {
                    Vsense = sensor;
                }
            }
        }

        // TODO
        //lift.MIN_LIFT_UP_PWR = Range.clip(0.2 + ((12.3 - Vsense.getVoltage())*0.1),0.1,0.3);

        //Add all components to an array list so they can be easily initialized
        components.add(drive);
        components.add(lift);
        components.add(claw);
    }

    /**
     * Initialize robot for Auto.
     */
    public void initAuto() {
        for (Component component : components)
            component.initAuto();
    }

    public void initTeleOp() {
        for (Component component : components)
            component.initTeleOp();
    }
}
