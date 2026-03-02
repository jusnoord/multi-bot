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
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotType;
import frc.robot.Constants.DemoConstants;
import frc.robot.Constants.RobotConfig;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Path;
import frc.robot.util.TunableNumber;

/**drives two robots in tandem */
public class FollowPath extends Command {
    private final Swerve swerve;

    private Path path;
    private final Pose2d finalPose;

    private StructEntry<Pose2d> targetPosePublisher;
    private StructSubscriber<Pose2d> masterPoseSubscriber;
    private BooleanSubscriber masterFinishedPathSub;
    private BooleanPublisher masterFinishedPathPub;


    //PID values
    private TunableNumber kP = new TunableNumber("tandem kP", RobotConstants.tandemkP);
    private TunableNumber kI = new TunableNumber("tandem kI", RobotConstants.tandemkI);
    private TunableNumber kD = new TunableNumber("tandem kD", RobotConstants.tandemkD);
    private TunableNumber kP_angle = new TunableNumber("tandem kP_angle", RobotConstants.tandemkP_angle);
    private TunableNumber kI_angle = new TunableNumber("tandem kI_angle", RobotConstants.tandemkI_angle);
    private TunableNumber kD_angle = new TunableNumber("tandem kD_angle", RobotConstants.tandemkD_angle);

    private final PIDController anglePIDRobot = new PIDController(RobotConstants.pathkP_angle, RobotConstants.pathkI_angle, RobotConstants.pathkD_angle);
    private final PIDController xVelocityPIDRobot = new PIDController(RobotConstants.velocitykP, RobotConstants.velocitykI, RobotConstants.velocitykD);
    private final PIDController yVelocityPIDRobot = new PIDController(RobotConstants.velocitykP, RobotConstants.velocitykI, RobotConstants.velocitykD);
    private final PIDController angleVelocityPIDRobot = new PIDController(RobotConstants.velocitykP_angle, RobotConstants.velocitykI_angle, RobotConstants.velocitykD_angle);

    private final PIDController anglePID = new PIDController(kP_angle.getDefault(), kI_angle.getDefault(), kD_angle.getDefault());
    private final PIDController xPID = new PIDController(kP.getDefault(), kI.getDefault(), kD.getDefault());
    private final PIDController yPID = new PIDController(kP.getDefault(), kI.getDefault(), kD.getDefault());

    private boolean robotAtTarget = false;
    private boolean PIDAtTolerance = false;

    public FollowPath(Swerve swerve, Path path, Pose2d finalPose) {
        this.swerve = swerve;
        this.path = path;
        this.finalPose = finalPose;

        masterPoseSubscriber = NetworkTableInstance.getDefault().getTable(Constants.RobotType.master.toString()).getStructTopic("RobotPose", Pose2d.struct).subscribe(new Pose2d(), Constants.NTPubSub);
        targetPosePublisher = NetworkTableInstance.getDefault().getTable(Constants.currentRobot.toString()).getStructTopic("targetPose", Pose2d.struct).getEntry(new Pose2d());

        BooleanTopic masterFinishedPath = NetworkTableInstance.getDefault().getTable(RobotType.master.toString()).getBooleanTopic("finished path");

        if(Constants.IS_MASTER) {
            masterFinishedPathPub = masterFinishedPath.publish(Constants.NTPubSub);
        } else {
            masterFinishedPathSub = masterFinishedPath.subscribe(false, Constants.NTPubSub);
        }

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        robotAtTarget = false;
        PIDAtTolerance = false;
        path.reset();
        // RobotConfig.reset();

        anglePID.enableContinuousInput(0, Math.PI * 2);
        anglePID.setTolerance(0.02);
        xPID.setTolerance(0.02);
        yPID.setTolerance(0.02);

        angleVelocityPIDRobot.enableContinuousInput(0, Math.PI * 2);
        angleVelocityPIDRobot.setTolerance(0.02);
        xVelocityPIDRobot.setTolerance(0.02);
        yVelocityPIDRobot.setTolerance(0.02);

        System.out.println("FollowPath started");
    }

    @Override
    public void execute() {

        Transform2d masterOffset = RobotConfig.offsetPositions[0];
        Transform2d slaveOffset = RobotConfig.offsetPositions[1];
        
        Pose2d masterPose = Constants.IS_MASTER ? swerve.getPose() : masterPoseSubscriber.get();
        Pose2d currentPose = swerve.getPose();
        Pose2d centerFormationPose = masterPose.plus(masterOffset.inverse());


        // formation velocity calculation
        boolean masterFinishedPath; 
        Pose2d formationVelocity;
        //if master, check if path is finished locally, else get from master robot
        if(Constants.IS_MASTER) {
            masterFinishedPath = path.isFinished();
            masterFinishedPathPub.accept(path.isFinished());
        } else {
            masterFinishedPath = masterFinishedPathSub.get();
        }

        if(!masterFinishedPath) {
            formationVelocity = path.getVelocity(centerFormationPose, anglePIDRobot); //if path not finished, continue
        } else {
            //else, get velocities from PID controllers to drive to final pose
            double xVeloc = MathUtil.clamp(xVelocityPIDRobot.calculate(centerFormationPose.getX(), finalPose.getX()), -RobotConstants.maxAutoDriveSpeedMetersPerSecond, RobotConstants.maxAutoDriveSpeedMetersPerSecond);
            double yVeloc = MathUtil.clamp(yVelocityPIDRobot.calculate(centerFormationPose.getY(), finalPose.getY()), -RobotConstants.maxAutoDriveSpeedMetersPerSecond, RobotConstants.maxAutoDriveSpeedMetersPerSecond);
            double angleVeloc = -MathUtil.clamp(angleVelocityPIDRobot.calculate(centerFormationPose.getRotation().getRadians(), finalPose.getRotation().getRadians()), -RobotConstants.maxAutoDriveAngularSpeedRadiansPerSecond, RobotConstants.maxAutoDriveAngularSpeedRadiansPerSecond);

            if(xPID.atSetpoint() && yPID.atSetpoint() && anglePID.atSetpoint()) {
                robotAtTarget = true;
                formationVelocity = new Pose2d();
            } else {
                formationVelocity = new Pose2d(xVeloc, yVeloc, new Rotation2d(angleVeloc));
            }

            System.out.println("Formation Velocity: " + formationVelocity);
        }

        Rotation2d rotationalSpeed = formationVelocity.getRotation();
        Translation2d translationalVelocity = formationVelocity.getTranslation(); 
        
        if (Constants.IS_MASTER) {
            Translation2d centerToMaster = currentPose.getTranslation().minus(centerFormationPose.getTranslation());
            double rotationalCompensationMagnitude = rotationalSpeed.getRadians() * masterOffset.getTranslation().getNorm();
            Rotation2d rotationalCompensationDirection = centerToMaster.getAngle().plus(Rotation2d.kCW_90deg);

            Translation2d rotationalCompensation = new Translation2d(rotationalCompensationMagnitude, rotationalCompensationDirection);

            Pose2d robotSpeeds = new Pose2d(translationalVelocity.plus(rotationalCompensation), rotationalSpeed);

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
            
            PIDAtTolerance = anglePID.atSetpoint() && xPID.atSetpoint() && yPID.atSetpoint();
            if (!PIDAtTolerance) {
                swerve.setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xOut, yOut, rOut, currentPose.getRotation()));
            } else {
                swerve.setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(robotSpeeds.getX(), robotSpeeds.getY(), robotSpeeds.getRotation().getRadians(), currentPose.getRotation()));
            }

            
            // update the PID values from the tunable numbers
            if(Constants.tuningMode) {
                anglePID.setPID(kP_angle.doubleValue(), kI_angle.doubleValue(), kD_angle.doubleValue());
                xPID.setPID(kP.doubleValue(), kI.doubleValue(), kD.doubleValue());
                yPID.setPID(kP.doubleValue(), kI.doubleValue(), kD.doubleValue());
            }

            //update telemetry
            targetPosePublisher.accept(robotTargetPose);
        }
    }

    // @Override
    // public void execute() {
    //     Transform2d offsetPosition = RobotConfig.offsetPositions[Constants.IS_MASTER ? 0 : 1];
    //     Pose2d masterPose = Constants.IS_MASTER ? swerve.getPose() : masterPoseSubscriber.get();
    //     Pose2d formationPose = masterPose.plus(RobotConfig.offsetPositions[0]); 
    //     Pose2d formationVelocity = path.getVelocity(formationPose, anglePIDRobot); 
    //     Pose2d robotVelocity = 
    //     Pose2d robotTargetPose = masterPose.plus(RobotConfig.offsetPositions[1].plus(RobotConfig.offsetPositions[0].inverse()));

    //     double xOut = robotVelocity.getX() + xPID.calculate(currentPose.getX(), robotTargetPose.getX());
    //     double yOut = robotVelocity.getY() + yPID.calculate(currentPose.getY(), robotTargetPose.getY());
    //     double rOut = robotVelocity.getRotation().getRadians() + anglePID.calculate(currentPose.getRotation().getRadians(), robotTargetPose.getRotation().getRadians());
        
    //     boolean PIDAtTolerance = anglePID.atSetpoint() && xPID.atSetpoint() && yPID.atSetpoint();
    //     if (!Constants.IS_MASTER && !PIDAtTolerance) {
    //         swerve.setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xOut, yOut, rOut, currentPose.getRotation()));
    //     } else {
    //         swerve.setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(robotVelocity.getX(), robotVelocity.getY(), robotVelocity.getRotation().getRadians(), currentPose.getRotation()));
    //     }

        
    //     // update the PID values from the tunable numbers
    //     if(Constants.tuningMode) {
    //         anglePID.setPID(kP_angle.doubleValue(), kI_angle.doubleValue(), kD_angle.doubleValue());
    //         xPID.setPID(kP.doubleValue(), kI.doubleValue(), kD.doubleValue());
    //         yPID.setPID(kP.doubleValue(), kI.doubleValue(), kD.doubleValue());
    //     }

    //     //update telemetry
    //     targetPosePublisher.accept(robotTargetPose);
    // }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        System.out.println("FollowPath ended");
    }

    @Override
    public boolean isFinished() {
        //if master, finish when at target
        //if slave, finish when at target and PID at tolerance
        return Constants.IS_MASTER ? robotAtTarget : PIDAtTolerance && robotAtTarget;
    }
}
