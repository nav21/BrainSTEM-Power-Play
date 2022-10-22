package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    // public Collector collector;
    // public Depositor depositor;



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

        //Get instance of hardware map
        HardwareMap map = opMode.hardwareMap;

        components = new ArrayList<>();

        //Initialize robot drive train
        drive = new BMecanumDrive(map);
        // PORTME collector = new Collector(map);
        // PORTME depositor = new Depositor(map);
        // PORTME lift = new Lift(map);
        // PORTME claw = new Claw(map);

        //Add all components to an array list so they can be easily initialized
        components.add(drive);
        // PORTME components.add(collector);
        // PORTME components.add(depositor);
        // PORTME components.add(lift);
        // PORTME components.add(claw);

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
