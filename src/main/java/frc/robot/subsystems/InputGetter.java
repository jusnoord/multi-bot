// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.DSSim;
import frc.robot.util.InputInterface;
import frc.robot.util.InputInterface.Inputs;

/**subsystem that runs on both robots to grab class from NT. 
 * Also handles enable/disable on slave */
public class InputGetter {
	private Inputs inputs;
    private DSSim dsSim;
    private boolean lastEnableStatus = false;
    private double lastTimestamp = 0;
    private final Thread updateThread;

    public InputGetter() {
    	inputs = InputInterface.grabInputs();
        if(!Constants.IS_MASTER) {
            dsSim = new DSSim();
            dsSim.init();
        }

        updateThread = new Thread(() -> {
            while (true) {
                periodic();
                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        updateThread.start();

    }

    public void periodic() {
    	inputs = InputInterface.grabInputs();

        if(!Constants.IS_MASTER) {
            //if the timestamp hasn't changed, then connection has been lost. disable no matter what.
            if(inputs.timeStamp == lastTimestamp) {
                dsSim.disable();
            } else if(inputs.isEnabled != lastEnableStatus) {
                if(inputs.isEnabled) {
                    dsSim.enable();
                } else {
                    dsSim.disable();
                }
                lastEnableStatus = inputs.isEnabled;
            }
        }

	}
    
    // public Pose2d getMasterOffset() {
    //     return inputs.masterOffset;
    // }

    public Pose2d getLeftJoystick() {
        return new Pose2d(-0.1 * getLeftY(), -0.1 * getLeftX(), Rotation2d.fromRadians(0.4 * getLeftTriggerAxis()));
    }
    public Pose2d getRightJoystick() {
        return new Pose2d(-0.1 * getRightY(), -0.1 * getRightX(), Rotation2d.fromRadians(0.4 * getRightTriggerAxis()));
    }

    public double getLeftX() {
        return inputs.leftX;
    }
    public double getLeftY() {
        return inputs.leftY;
    }
    public double getRightX() {
        return inputs.rightX;
    }
    public double getRightY() {
        return inputs.rightY;
    }
    public double getLeftTriggerAxis() {
        return inputs.leftTrigger;
    }
    public double getRightTriggerAxis() {
        return inputs.rightTrigger;
    }
    public int getPOV() {
        return inputs.pov;
    }
    public boolean getAButton() {
        return inputs.aButton;
    }
    public boolean getBButton() {
        return inputs.bButton;
    }
    public boolean getXButton() {
        return inputs.xButton;
    }
    public boolean getYButton() {
        return inputs.yButton;
    }
    public boolean getLeftBumper() {
        return inputs.leftBumper;
    }
    public boolean getRightBumper() {
        return inputs.rightBumper;
    }
    public boolean getStartButton() {
        return inputs.startButton;
    }
    public boolean getBackButton() {
        return inputs.backButton;
    }
    public Pose2d getJoystickVelocity() {
        return inputs.joystickVelocity;
    }
}
