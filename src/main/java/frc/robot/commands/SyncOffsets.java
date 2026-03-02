// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotConfig;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;

/**
 * drives a given robot independently
 */
public class SyncOffsets extends Command {
  public static Transform2d masterOffset = RobotConfig.offsetPositions[0];
  private final Swerve swerve;
  
  private StructSubscriber<Pose2d> masterPoseSubscriber;
  private StructSubscriber<Transform2d> masterOffsetSubscriber, slaveOffsetSubscriber;
  private StructPublisher<Pose2d> centerPosePublisher, slaveTargetPosePublisher, masterCurrentPosePublisher, slaveCurrentPosePublisher;
  private StructPublisher<Transform2d> masterOffsetPublisher, slaveOffsetPublisher;
  
  public SyncOffsets(Swerve swerve) {
    this.swerve = swerve;
    if (!Constants.IS_MASTER) {
      centerPosePublisher = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("center pose", Pose2d.struct).publish();       
      masterOffsetPublisher = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("master offset", Transform2d.struct).publish();       
      slaveOffsetPublisher = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("slave offset", Transform2d.struct).publish();       
      slaveTargetPosePublisher = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("slave target", Pose2d.struct).publish();       
      masterCurrentPosePublisher = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("master pose", Pose2d.struct).publish();       
      slaveCurrentPosePublisher = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("slave pose", Pose2d.struct).publish();       
      masterPoseSubscriber = NetworkTableInstance.getDefault().getTable(Constants.RobotType.master.toString()).getStructTopic("RobotPose", Pose2d.struct).subscribe(new Pose2d());
    } else {
      masterOffsetSubscriber = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.getOpposite().toString()).getStructTopic("master offset", Transform2d.struct).subscribe(new Transform2d(-1, -1, new Rotation2d()));       
      slaveOffsetSubscriber = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.getOpposite().toString()).getStructTopic("slave offset", Transform2d.struct).subscribe(new Transform2d(-1, -1, new Rotation2d()));       
      masterOffsetPublisher = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("master offset", Transform2d.struct).publish();     // telemetry only  
      slaveOffsetPublisher = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("slave offset", Transform2d.struct).publish();       // telemetry only

    }
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    System.out.println("SyncOffsets initialized");
  }

  @Override
  public void execute() {
    if (!Constants.IS_MASTER) {
      Pose2d masterPose = masterPoseSubscriber.get();
      Pose2d currentPose = swerve.getPose();
      Pose2d formationCenterPose = new Pose2d(masterPose.getTranslation().plus(currentPose.getTranslation()).times(0.5), masterPose.getRotation());
      masterOffset = new Transform2d(masterPose.getTranslation().minus(formationCenterPose.getTranslation()).rotateBy(masterPose.getRotation().times(-1)), new Rotation2d());
      Transform2d slaveOffset = new Transform2d(currentPose.getTranslation().minus(formationCenterPose.getTranslation()).rotateBy(masterPose.getRotation().times(-1)), currentPose.getRotation().minus(formationCenterPose.getRotation()));
      RobotConfig.offsetPositions[1] = slaveOffset;
      RobotConfig.offsetPositions[0] = masterOffset;
 
      centerPosePublisher.accept(formationCenterPose);
      masterOffsetPublisher.accept(masterOffset);
      slaveOffsetPublisher.accept(slaveOffset);

      slaveTargetPosePublisher.accept(masterPose.plus(RobotConfig.offsetPositions[1].plus(RobotConfig.offsetPositions[0].inverse())));
      masterCurrentPosePublisher.accept(masterPose);
      slaveCurrentPosePublisher.accept(currentPose);
    } else {
      RobotConfig.offsetPositions[1] = slaveOffsetSubscriber.get();
      RobotConfig.offsetPositions[0] = masterOffsetSubscriber.get();
      masterOffsetPublisher.accept(RobotConfig.offsetPositions[0]);// telemetry only
      slaveOffsetPublisher.accept(RobotConfig.offsetPositions[1]); // telemetry only
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("SyncOffsets ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
