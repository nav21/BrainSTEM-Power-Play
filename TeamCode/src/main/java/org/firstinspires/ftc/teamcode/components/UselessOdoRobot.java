package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.utils.BotLog;
import org.firstinspires.ftc.teamcode.utils.Component;

import java.util.ArrayList;

/**
 * Created by parvs on 7/26/2018.
 */

public class UselessOdoRobot {
    //Various components of the TeleOp robot
    public BMecanumDrive drive;

    //List of components to be initialized
    private final ArrayList<Component> components;

    /**
     * Instantiates a new TeleOp robot.
     *
     * @param opMode the op mode
     */
    public UselessOdoRobot(LinearOpMode opMode) {
        //Get instance of hardware map and telemetry
        HardwareMap map = opMode.hardwareMap;
        components = new ArrayList<>();

        //Initialize robot components
        drive = new BMecanumDrive(map);




        // TODO
        //lift.MIN_LIFT_UP_PWR = Range.clip(0.2 + ((12.3 - Vsense.getVoltage())*0.1),0.1,0.3);

        //Add all components to an array list so they can be easily initialized
        components.add(drive);
    }

    /**
     * Initialize robot for Auto.
     */

}
