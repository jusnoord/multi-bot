package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.LiftConstants.LiftPosition;

public class Lift extends SubsystemBase {
    private TalonFX liftMotor;
    private CANrange canRange;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private PositionDutyCycle pid;
    private double targetPosition = 0;

    private DoublePublisher liftPose;
    private DoublePublisher canRangePub;

    public Lift() {
        liftMotor = makeMotor(RobotMap.liftMotorID, RobotMap.liftInvert, RobotMap.liftBrake, RobotMap.liftStatorLimit, RobotMap.liftEncoderToMechanismRatio, RobotMap.liftRotorToEncoderRatio, RobotMap.liftkP, RobotMap.liftkI, RobotMap.liftkD);
        canRange = new CANrange(RobotMap.canRangeID, "*"); //default config should be fine idk, might wanna decrease FOV

        liftPose = NetworkTableInstance.getDefault().getTable("Lift").getSubTable(Constants.currentRobot.toString()).getDoubleTopic("Pose").publish();
        canRangePub = NetworkTableInstance.getDefault().getTable("Lift").getSubTable(Constants.currentRobot.toString()).getDoubleTopic("CanRange").publish();

        resetEncoder();
        pid = new PositionDutyCycle(getPosition());
    }

    public TalonFX makeMotor(int id, boolean inverted, boolean isBrake, double statorLimit, double ENCODER_TO_MECHANISM_RATIO, double ROTOR_TO_ENCODER_RATIO, double kP, double kI, double kD) {
        TalonFX motor = new TalonFX(id, "*");

        // //set neutral mode and inverts
        // config.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        // config.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        // //set current limits; supply current limits are hardcoded because they are almost always the same
        // config.CurrentLimits.SupplyCurrentLimitEnable = true;
        // config.CurrentLimits.SupplyCurrentLimit = 40d;

        // config.CurrentLimits.StatorCurrentLimitEnable = statorLimit > 0;
        // config.CurrentLimits.StatorCurrentLimit = statorLimit;

        // //set feedback ratios
        // config.Feedback.SensorToMechanismRatio = ENCODER_TO_MECHANISM_RATIO;
        // config.Feedback.RotorToSensorRatio = ROTOR_TO_ENCODER_RATIO;



        config.MotorOutput.PeakForwardDutyCycle = 0.2;
        config.MotorOutput.PeakReverseDutyCycle = -0.2;


        config.CurrentLimits.StatorCurrentLimitEnable = 30 > 0;
        config.CurrentLimits.StatorCurrentLimit = 30;

        //set feedback ratios
        config.Feedback.SensorToMechanismRatio = 50/(Inches.of(1.25).in(Centimeters) * Math.PI);
        config.Feedback.RotorToSensorRatio = 0;

        config.MotorOutput.Inverted = Constants.IS_MASTER ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 1d/12d;
        config.Slot0.kI = 0d;
        config.Slot0.kD = 0d;

        //TODO: might need other slots?

        motor.getConfigurator().apply(config);

        return motor;
    }

    public void setPower(double power) {
        liftMotor.setControl(new DutyCycleOut(power));
    }

    public double getPosition() {
        return liftMotor.getPosition().getValueAsDouble();
    }

    /**
     * Gets the distance to the target from the CANrange in centimeters
     * @return zero when elevator is bottomed out
     */
    public double getCanRangeDistance() {
        return (canRange.getDistance().getValueAsDouble() * 100.0) - LiftConstants.canRangeOffset;
    }

    /**
     * Sets the position of the lift in centimeters using the CANrange as feedback. Zero is when the elevator is bottomed out.
     * @param position in centimeters (ideally)
     */
    public void setPosition(double position) {
        liftMotor.setControl(pid.withPosition(position));
        targetPosition = position;
    }

    public void setPosition(LiftPosition position) {
        setPosition(position.height);
    }

    public boolean onTarget() {
        return Math.abs(targetPosition - getPosition()) < LiftConstants.positionTolerance;
    }

    public void resetEncoder() {
        liftMotor.setPosition(getCanRangeDistance());
    }

    /**
     * Returns a command that sets the lift to the given position and finishes when it reaches the target.
     * @param position lift state to move to
     */
    public Command setLiftState(LiftPosition position) {
        return new Command() {
            @Override
            public void execute() {
                setPosition(position);
            }

            @Override
            public boolean isFinished() {
                return onTarget();
            }
        };
    }

    public void stop() {
        liftMotor.stopMotor();
    }

    @Override
    public void periodic() {
        liftPose.accept(getPosition());
        canRangePub.accept(getCanRangeDistance());
    }
}
