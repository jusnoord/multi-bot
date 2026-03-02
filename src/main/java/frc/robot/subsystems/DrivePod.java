// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Represents a single NERDSwerve drive pod.
 * Handles motor and encoder configuration, state control, and pod operations.
 */
public class DrivePod extends SubsystemBase {
    public CANcoder absoluteEncoder;
    public TalonFX azimuthMotor;
    public TalonFX driveMotor;
    private double speed = 0;
    TalonFX motor;


    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    TalonFXConfiguration azimuthConfig = new TalonFXConfiguration();
    CANcoderConfiguration coderConfig = new CANcoderConfiguration();

    //variable that determines whether or not to apply PID configurations to the motor (defaults to true for initial application)
    boolean apply = true;

    //making position duty cycle default because it's the simplest. if you want to use a different control type, you can change it
    private final PositionVoltage anglePID = new PositionVoltage(0).withSlot(0);


    private double initialOffset;


    public DrivePod(int absoluteEncoderID, int azimuthID, int driveID, double absoluteEncoderOffset, boolean azimuthInvert, int azimuthLimit, double azimuthRotationsPerRot, boolean azimuthBrake, double azimuthRR, double kP, double kI, double kD, double kS, double kV, double kA, double maxOut, double ADMult, boolean driveInvert, int driveLimit, boolean driveBrake, double driveRR) {
        // System.out.println("drivePod called");
        absoluteEncoder = makeCANCoder(absoluteEncoderID, false, absoluteEncoderOffset);
        
        driveMotor = makeDrive(driveID, driveInvert, driveBrake, driveLimit, driveRR, Constants.RobotConfig.driveMetersPerMotorRotation, 1d);
        azimuthMotor = makeAzimuth(azimuthID, absoluteEncoderID, azimuthInvert, azimuthBrake, azimuthLimit, azimuthRR, 1d, azimuthRotationsPerRot);
        

        if(azimuthConfig.Slot0.kP != kP) {azimuthConfig.Slot0.kP = kP; apply = true;}
        if(azimuthConfig.Slot0.kI != kI) {azimuthConfig.Slot0.kI = kI; apply = true;}
        if(azimuthConfig.Slot0.kD != kD) {azimuthConfig.Slot0.kD = kD; apply = true;}
        if(azimuthConfig.Slot0.kS != kS) {azimuthConfig.Slot0.kS = kS; apply = true;}
        if(azimuthConfig.Slot0.kV != kV) {azimuthConfig.Slot0.kV = kV; apply = true;}
        if(azimuthConfig.Slot0.kA != kA) {azimuthConfig.Slot0.kA = kA; apply = true;}
        if(azimuthConfig.MotorOutput.PeakForwardDutyCycle != maxOut) {
            azimuthConfig.MotorOutput.PeakForwardDutyCycle = maxOut;
            azimuthConfig.MotorOutput.PeakReverseDutyCycle = -maxOut;
            apply = true;
        }
        if(apply) {
            this.azimuthMotor.getConfigurator().apply(azimuthConfig);
        } 

        apply = false;
        
        resetPod();
    }

    /**
     * Creates a new TurdonFX (please use this for drive motors only)
     * @param id CAN ID for the motor
     * @param inverted true for CW+, false for CCW+
     * @param isBrake true for brake, false for coast
     * @param statorLimit the stator current limit in amps
     * @param rampRate time it takes for the motor to reach full power from zero power in seconds
     * @param ENCODER_TO_MECHANISM_RATIO ratio between the feedback encoder (integrated for drive motors) and the mechanism. this varies based on your gear ratio
     * @param ROTOR_TO_ENCODER_RATIO ratio between the rotor and the feedback encoder. this is usually 1 for drive motors
     */
    public TalonFX makeDrive(int id, boolean inverted, boolean isBrake, double statorLimit, double rampRate, double ENCODER_TO_MECHANISM_RATIO, double ROTOR_TO_ENCODER_RATIO) {
        //I figured nobody had the guts to put a CANivore on a turdswerve, so i'm leaving out the CAN bus parameter
        motor = new TalonFX(id, "*");

        //set neutral mode and inverts
        driveConfig.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        //set current limits; supply current limits are hardcoded because they are almost always the same
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = 40d;

        driveConfig.CurrentLimits.StatorCurrentLimitEnable = statorLimit > 0;
        driveConfig.CurrentLimits.StatorCurrentLimit = statorLimit;


        // this is kind of bad code, but it's the easiest way to set a ramp rate regardless of control type
        driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampRate;
        driveConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = rampRate;
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = rampRate; 
        driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = rampRate;
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = rampRate;
        driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = rampRate;

        //set feedback ratios
        driveConfig.Feedback.SensorToMechanismRatio = ENCODER_TO_MECHANISM_RATIO;
        driveConfig.Feedback.RotorToSensorRatio = ROTOR_TO_ENCODER_RATIO;
        //the remote sensor defaults to internal encoder

        // System.out.print("drive made");
        motor.getConfigurator().apply(driveConfig);

        return motor;
    }

    /**
     * Creates a new TurdonFX (please use this for azimuth with fused CANcoders only)
     * @param id CAN ID for the motor
     * @param inverted true for CW+, false for CCW+
     * @param isBrake true for brake, false for coast
     * @param statorLimit the stator current limit in amps
     * @param rampRate time it takes for the motor to reach full power from zero power in seconds
     * @param encoderID the id of the CANcoder used fused with the motor
     * @param ENCODER_TO_MECHANISM_RATIO Ratio between the feedback encoder (CANcoder) and mechanism. This is usually 1 for azimuth motors.
     * @param ROTOR_TO_ENCODER_RATIO Ratio between the rotor and the feedback encoder. This is depends on the gearbox.
     * 
     * this constructor is for azimuth motors only and uses fused CANcoders. If you are not using CANcoders or do not have phoenix pro, please use another constructor
     */
    public TalonFX makeAzimuth(int id, int encoderID, boolean inverted, boolean isBrake, double statorLimit, double rampRate, double ENCODER_TO_MECHANISM_RATIO, double ROTOR_TO_ENCODER_RATIO) {
        //I figured nobody had the guts to put a CANivore on a turdswerve, so i'm leaving out the CAN bus parameter
        motor = new TalonFX(id, "*");

        //set neutral mode and inverts
        azimuthConfig.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        azimuthConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        //set current limits; supply current limits are hardcoded because they are almost always the same
        azimuthConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        azimuthConfig.CurrentLimits.SupplyCurrentLimit = 40d;

        azimuthConfig.CurrentLimits.StatorCurrentLimitEnable = statorLimit > 0;
        azimuthConfig.CurrentLimits.StatorCurrentLimit = statorLimit;


        // this is kind of bad code, but it's the easiest way to set a ramp rate regardless of control type
        azimuthConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampRate;
        azimuthConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = rampRate;
        azimuthConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = rampRate; 
        azimuthConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = rampRate;
        azimuthConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = rampRate;
        azimuthConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = rampRate;

        //set feedback ratios
        azimuthConfig.Feedback.SensorToMechanismRatio = ENCODER_TO_MECHANISM_RATIO;
        azimuthConfig.Feedback.RotorToSensorRatio = ROTOR_TO_ENCODER_RATIO;

        azimuthConfig.ClosedLoopGeneral.ContinuousWrap = true;
        azimuthConfig.Feedback.FeedbackRemoteSensorID = encoderID;
        azimuthConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        // System.out.print("azimuth made");
        motor.getConfigurator().apply(azimuthConfig);

        return motor;
    }

    /**
     * creates a new CANTurdCoder
     * @param inverted true for CW+, false for CCW+
     * @param offset the offset of the sensor in rotations
     * @param id the CAN id of the sensor
     */
    private CANcoder makeCANCoder(int id, boolean inverted, double offset) {
        CANcoder encoder = new CANcoder(id, "*");

        coderConfig.MagnetSensor.SensorDirection = inverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        coderConfig.MagnetSensor.MagnetOffset = offset;

        //not applying magnet config directly in order to overwrite other settings
        encoder.getConfigurator().apply(coderConfig);
        
        initialOffset = offset;

        return encoder;
    }

    public void setPID(double P, double I, double D) {
        if(azimuthConfig.Slot0.kP != P) {azimuthConfig.Slot0.kP = P; apply = true;}
        if(azimuthConfig.Slot0.kI != I) {azimuthConfig.Slot0.kI = I; apply = true;}
        if(azimuthConfig.Slot0.kD != D) {azimuthConfig.Slot0.kD = D; apply = true;}
        if(apply) {
            azimuthMotor.getConfigurator().apply(azimuthConfig);
        } 
        apply = false;
    }

    public void resetPod() {
        driveMotor.setPosition(0);
        azimuthMotor.setPosition(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void revertZero() {
        if(coderConfig.MagnetSensor.MagnetOffset != initialOffset) {
            coderConfig.MagnetSensor.MagnetOffset = initialOffset; 
            absoluteEncoder.getConfigurator().apply(coderConfig);
        }
        resetPod();
    }
    
    public void stop() {
        azimuthMotor.set(0);
        driveMotor.set(0);
    }

    public SwerveModulePosition getPodPosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), Rotation2d.fromRotations(azimuthMotor.getPosition().getValueAsDouble()));
    }
    
	public SwerveModuleState getState() {
		return new SwerveModuleState(getVelocity(),
				Rotation2d.fromRotations(getAngle()));
	}

    public double getVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    public double getAngle() {
        return azimuthMotor.getPosition().getValueAsDouble();
    }

	public void setPodState(SwerveModuleState state) {
		setPodState(state, true);
	}

    public void setRotationalSpeed(double speed) {
        azimuthMotor.set(speed);
    }

    public void setPodState(SwerveModuleState state, boolean isTurningEnabled) {
        state.optimize(Rotation2d.fromRotations(getAngle())); // does not account for rotations between 180 and 360?
        azimuthMotor.setControl(anglePID.withPosition(state.angle.getRotations())); 
        speed = state.speedMetersPerSecond;

        driveMotor.set(speed); 
    }

    @Override
    public void periodic() {        
        // SmartDashboard.putNumber("absolute encoder" + absoluteEncoder.getDeviceID(), absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        // SmartDashboard.putNumber("azimuth pose " + absoluteEncoder.getDeviceID(), azimuthMotor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("azimuth pose " + config.absoluteEncoderID, azimuthMotor.);

        // SmartDashboard.putNumber("drive pos " + driveMotor.getDeviceId(), driveEncoder.getPosition());
        // SmartDashboard.putNumber("azimuth.getAppliedOutput()" + azimuthMotor.getDeviceId(), azimuthMotor.getAppliedOutput()); //getAppliedOutput());
    }
}
