// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConfig;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.util.TunableNumber;

/**drives two robots in tandem */
public class AutoDrive extends Command {
    private final Swerve swerve;
    private Supplier<Pose2d> targetPose;


    private StructEntry<Pose2d> targetPosePublisher;
    private StructSubscriber<Pose2d> masterPoseSubscriber;

    private StructPublisher<Pose2d> robotSpeedPublisher;

    private BooleanPublisher atTargetPublisher;
    private boolean PIDAtTolerance = false;



    //PID values
    // private TunableNumber kP = new TunableNumber("autonomous kP", RobotConstants.autoDrivekP);
    // private TunableNumber kI = new TunableNumber("autonomous kI", RobotConstants.autoDrivekI);
    // private TunableNumber kD = new TunableNumber("autonomous kD", RobotConstants.autoDrivekD);
    // private TunableNumber kP_angle = new TunableNumber("autonomous kP_angle", RobotConstants.autoDrivekP_angle);
    // private TunableNumber kI_angle = new TunableNumber("autonomous kI_angle", RobotConstants.autoDrivekI_angle);
    // private TunableNumber kD_angle = new TunableNumber("autonomous kD_angle", RobotConstants.autoDrivekD_angle);

    private final PIDController anglePID;
    private final PIDController xPID;
    private final PIDController yPID;

    public AutoDrive(Swerve swerve, Supplier<Pose2d> targetPose, boolean lowAccuracy) {
        this.swerve = swerve;
        this.targetPose = targetPose;
        masterPoseSubscriber = NetworkTableInstance.getDefault().getTable(Constants.RobotType.master.toString()).getStructTopic("RobotPose", Pose2d.struct).subscribe(new Pose2d());
        robotSpeedPublisher = NetworkTableInstance.getDefault().getTable(Constants.currentRobot.toString()).getStructTopic("desired tandem speed", Pose2d.struct).publish();
        atTargetPublisher = NetworkTableInstance.getDefault().getTable(Constants.currentRobot.toString()).getBooleanTopic("autodrive at target").publish();
        
        double kP, kI, kD, anglekP, anglekI, anglekD, angleTolerance, positionTolerance;

        kP = RobotConstants.autoDrivekP;
        kI = RobotConstants.autoDrivekI;
        kD = RobotConstants.autoDrivekD;

        anglekP = RobotConstants.autoDrivekP_angle;
        anglekI = RobotConstants.autoDrivekI_angle;
        anglekD = RobotConstants.autoDrivekD_angle;

        if(lowAccuracy) {
            angleTolerance = RobotConstants.highAutoDriveAngleTolerance;
            positionTolerance = RobotConstants.highAutoDrivePositionTolerance;
        } else {        
            angleTolerance = RobotConstants.lowAutoDriveAngleTolerance;
            positionTolerance = RobotConstants.lowAutoDrivePositionTolerance;
        }

        xPID = new PIDController(kP, kI, kD);
        yPID = new PIDController(kP, kI, kD);
        anglePID = new PIDController(anglekP, anglekI, anglekD);

        anglePID.setTolerance(angleTolerance);
        xPID.setTolerance(positionTolerance);
        yPID.setTolerance(positionTolerance);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        System.out.println("AutoDrive started");
        anglePID.enableContinuousInput(0, Math.PI * 2);

        //initialize telemetry
        targetPosePublisher = NetworkTableInstance.getDefault().getTable(Constants.currentRobot.toString()).getStructTopic("targetPose", Pose2d.struct).getEntry(new Pose2d());
        atTargetPublisher.accept(false);
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();

        Pose2d robotTargetPose = targetPose.get();

        //calculate the speeds to drive towards the target pose
        double xOut = xPID.calculate(currentPose.getX(), robotTargetPose.getX());
        double yOut = yPID.calculate(currentPose.getY(), robotTargetPose.getY());
        double rOut = -anglePID.calculate(currentPose.getRotation().getRadians(), robotTargetPose.getRotation().getRadians());
        
        PIDAtTolerance = anglePID.atSetpoint() && xPID.atSetpoint() && yPID.atSetpoint();

        //clamp the outputs to max speeds
        xOut = MathUtil.clamp(xOut, -RobotConstants.maxAutoDriveSpeedMetersPerSecond, RobotConstants.maxAutoDriveSpeedMetersPerSecond);
        yOut = MathUtil.clamp(yOut, -RobotConstants.maxAutoDriveSpeedMetersPerSecond, RobotConstants.maxAutoDriveSpeedMetersPerSecond);
        rOut = MathUtil.clamp(rOut, -RobotConstants.maxAutoDriveAngularSpeedRadiansPerSecond, RobotConstants.maxAutoDriveAngularSpeedRadiansPerSecond);

        swerve.setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xOut, yOut, rOut, currentPose.getRotation()));
        
        // update the PID values from the tunable numbers
        // if(Constants.tuningMode) {
        //     anglePID.setPID(kP_angle.doubleValue(), kI_angle.doubleValue(), kD_angle.doubleValue());
        //     xPID.setPID(kP.doubleValue(), kI.doubleValue(), kD.doubleValue());
        //     yPID.setPID(kP.doubleValue(), kI.doubleValue(), kD.doubleValue());
        // }

        //update telemetry
        targetPosePublisher.accept(robotTargetPose);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AutoDrive ended");
        atTargetPublisher.accept(true);
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return PIDAtTolerance;
    }
}
