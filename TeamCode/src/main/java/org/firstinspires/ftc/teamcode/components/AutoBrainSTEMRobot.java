package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.Component;
import org.firstinspires.ftc.teamcode.components.Lift;
import org.firstinspires.ftc.teamcode.components.Claw;
import org.firstinspires.ftc.teamcode.components.BMecanumDrive;

import java.util.ArrayList;

/**
 * Created by parvs on 7/26/2018.
 */

public class AutoBrainSTEMRobot implements Runnable
{
    //Various components of the autonomous robot
    public BMecanumDrive drive;
    public Claw claw;
    public Lift lift;
    public VoltageSensor Vsense;

    private Thread updateThread;
    private boolean started = false;

    //Instance of linear opmode to use for hwMap
    private LinearOpMode opMode;

    //List of components to be initialized
    private ArrayList<Component> components;

    /**
     * Instantiates a new Autonomous robot.
     *
     * @param opMode the op mode
     */
    public AutoBrainSTEMRobot(LinearOpMode opMode)
    {
        this.opMode = opMode;

        //Get instance of hardware map and telemetry
        HardwareMap map = opMode.hardwareMap;

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

        lift.MIN_LIFT_UP_PWR = Range.clip(0.2 + ((12.3 - Vsense.getVoltage())*0.1),0.1,0.3);

        //Add all components to an array list so they can be easily initialized
        components.add(drive);
        components.add(lift);
        components.add(claw);
    }

    public void start()
    {
        if (!started)
        {
            updateThread = new Thread(this);
            updateThread.start();
            updateThread.setPriority(Thread.MAX_PRIORITY);
            started = true;
        }
    }

    @Override
    public synchronized void run()
    {
        while (started)
        {
            for (Component component : components)
                if (!opMode.isStopRequested())
                    component.update();
                else
                    stop();
        }
    }

    public void stop()
    {
        started = false;
    }

    /**
     * Initialize robot for autonomous.
     */
    public void initBlockAuto()
    {
        for (Component component : components)
            component.initBlockAuto();
    }

    public String test()
    {
        String failures = "";
        for (Component component : components)
            failures += component.test();
        return failures;
    }
}
