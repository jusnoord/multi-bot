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
public class TandemDrive extends Command {
    private final Swerve swerve;
    private Supplier<Pose2d> joystickVelocity;


    private StructEntry<Pose2d> targetPosePublisher;
    private StructSubscriber<Pose2d> masterPoseSubscriber;

    private StructPublisher<Pose2d> robotSpeedPublisher;



    //PID values
    private TunableNumber kP = new TunableNumber("tandem kP", RobotConstants.tandemkP);
    private TunableNumber kI = new TunableNumber("tandem kI", RobotConstants.tandemkI);
    private TunableNumber kD = new TunableNumber("tandem kD", RobotConstants.tandemkD);
    private TunableNumber kP_angle = new TunableNumber("tandem kP_angle", RobotConstants.tandemkP_angle);
    private TunableNumber kI_angle = new TunableNumber("tandem kI_angle", RobotConstants.tandemkI_angle);
    private TunableNumber kD_angle = new TunableNumber("tandem kD_angle", RobotConstants.tandemkD_angle);

    private TunableNumber tolerance = new TunableNumber("tandem tolerance", RobotConstants.tandemPositionTolerance);
    private TunableNumber angleTolerance = new TunableNumber("tandem angle tolerance", RobotConstants.tandemAngleTolerance);


    private final PIDController anglePID = new PIDController(kP_angle.getDefault(), kI_angle.getDefault(), kD_angle.getDefault());
    private final PIDController xPID = new PIDController(kP.getDefault(), kI.getDefault(), kD.getDefault());
    private final PIDController yPID = new PIDController(kP.getDefault(), kI.getDefault(), kD.getDefault());

    public TandemDrive(Swerve swerve, Supplier<Pose2d> joystickVelocity) {
        this.swerve = swerve;
        this.joystickVelocity = joystickVelocity;
        masterPoseSubscriber = NetworkTableInstance.getDefault().getTable(Constants.RobotType.master.toString()).getStructTopic("RobotPose", Pose2d.struct).subscribe(new Pose2d());
        robotSpeedPublisher = NetworkTableInstance.getDefault().getTable(Constants.currentRobot.toString()).getStructTopic("desired tandem speed", Pose2d.struct).publish();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        anglePID.enableContinuousInput(0, Math.PI * 2);
        anglePID.setTolerance(angleTolerance.getDefault());
        xPID.setTolerance(tolerance.getDefault());
        yPID.setTolerance(tolerance.getDefault());

        //initialize telemetry
        targetPosePublisher = NetworkTableInstance.getDefault().getTable(Constants.currentRobot.toString()).getStructTopic("targetPose", Pose2d.struct).getEntry(new Pose2d());
    }

    @Override
    public void execute() {
        Transform2d masterOffset = RobotConfig.offsetPositions[0];
        Transform2d slaveOffset = RobotConfig.offsetPositions[1];

        Pose2d currentPose = swerve.getPose();
        Pose2d masterPose = Constants.IS_MASTER ? currentPose : masterPoseSubscriber.get();
        Pose2d centerFormationPose = masterPose.plus(masterOffset.inverse());

        Rotation2d rotationalSpeed = joystickVelocity.get().getRotation();
        Translation2d translationalVelocity = joystickVelocity.get().getTranslation(); 
        
        if (Constants.IS_MASTER) {
            Translation2d centerToMaster = currentPose.getTranslation().minus(centerFormationPose.getTranslation());
            double rotationalCompensationMagnitude = rotationalSpeed.getRadians() * masterOffset.getTranslation().getNorm();
            Rotation2d rotationalCompensationDirection = centerToMaster.getAngle().plus(Rotation2d.kCW_90deg);

            Translation2d rotationalCompensation = new Translation2d(rotationalCompensationMagnitude, rotationalCompensationDirection);

            Pose2d robotSpeeds = new Pose2d(translationalVelocity.plus(rotationalCompensation), rotationalSpeed);
            robotSpeedPublisher.accept(new Pose2d(new Translation2d(), rotationalCompensationDirection));

            swerve.setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(robotSpeeds.getX(), robotSpeeds.getY(), robotSpeeds.getRotation().getRadians(), currentPose.getRotation()));

        } else {
            Translation2d centerToSlave = currentPose.getTranslation().minus(centerFormationPose.getTranslation());

            double rotationalCompensationMagnitude = rotationalSpeed.getRadians() * slaveOffset.getTranslation().getNorm();
            Rotation2d rotationalCompensationDirection = centerToSlave.getAngle().plus(Rotation2d.kCW_90deg);

            Translation2d rotationalCompensation = new Translation2d(rotationalCompensationMagnitude, rotationalCompensationDirection);
            Pose2d robotSpeeds = new Pose2d(rotationalCompensation.plus(translationalVelocity), rotationalSpeed);

            // new Pose2d(joystickVelocity.get().getTranslation().rotateBy(masterPose.getRotation())
            //     .plus(new Translation2d(joystickVelocity.get().getRotation().getRadians() * offsetPosition.getTranslation().getNorm(), 
            //       offsetPosition.getTranslation().getAngle().plus(Constants.IS_MASTER ? Rotation2d.kZero : Rotation2d.kCCW_90deg).plus(offsetPosition.getRotation()).plus(masterPose.getRotation()))), 
            // joystickVelocity.get().getRotation());

            //grab the target pose calculated by the master robot, add it onto the offset position for this robot
            Pose2d robotTargetPose = centerFormationPose.plus(RobotConfig.offsetPositions[1]);

            //calculate the speeds to drive towards the target pose
            double xOut = robotSpeeds.getX() + xPID.calculate(currentPose.getX(), robotTargetPose.getX());
            double yOut = robotSpeeds.getY() + yPID.calculate(currentPose.getY(), robotTargetPose.getY());
            double rOut = robotSpeeds.getRotation().getRadians() - anglePID.calculate(currentPose.getRotation().getRadians(), robotTargetPose.getRotation().getRadians());
            
            boolean PIDAtTolerance = anglePID.atSetpoint() && xPID.atSetpoint() && yPID.atSetpoint();
            if (!PIDAtTolerance) {
                swerve.setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xOut, yOut, rOut, currentPose.getRotation()));
            } else {
                swerve.setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(robotSpeeds.getX(), robotSpeeds.getY(), robotSpeeds.getRotation().getRadians(), currentPose.getRotation()));
            }

            
            // update the PID values from the tunable numbers
            if(true) {
                anglePID.setPID(kP_angle.doubleValue(), kI_angle.doubleValue(), kD_angle.doubleValue());
                xPID.setPID(kP.doubleValue(), kI.doubleValue(), kD.doubleValue());
                yPID.setPID(kP.doubleValue(), kI.doubleValue(), kD.doubleValue());

                anglePID.setTolerance(angleTolerance.doubleValue());
                xPID.setTolerance(tolerance.doubleValue());
                yPID.setTolerance(tolerance.doubleValue());
            }
            robotSpeedPublisher.accept(new Pose2d(new Translation2d(), rotationalCompensationDirection));

            //update telemetry
            targetPosePublisher.accept(robotTargetPose);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
