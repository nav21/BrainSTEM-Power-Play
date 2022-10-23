package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    // PORTME public Collector collector;
    // PORTME public Depositor depositor;

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

        components = new ArrayList<>();

        //Initialize robot components
        drive = new BMecanumDrive(map);


       lift = new Lift(map);

        claw = new Claw(map);

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
