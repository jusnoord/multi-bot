// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/** 
 * manually select a single pod and command a rotational speed 
 * This command is used to spin a swerve pod manually for testing purposes.
 */
public class SpinManually extends Command {
    private Swerve swerve;
    private Supplier<Integer> pod;
    private Supplier<Double> command;
    
    /** 
     * manually select a single pod and command a rotational speed 
     * @param swerve the swerve subsystem
     * @param pod a supplier that returns the pod index to control, or -1 to stop
     * @param command a supplier that returns the rotational speed to set the pod to
    */
    public SpinManually(Swerve swerve, Supplier<Integer> pod, Supplier<Double> command) {
        this.swerve = swerve;
        this.pod = pod;
        this.command = command;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.stop();
    }

    @Override
    public void execute() {
        if (pod.get() != -1) {
            swerve.getPods().get(pod.get()).setRotationalSpeed(command.get());
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return pod.get() == -1;
    }
}
