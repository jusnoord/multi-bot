// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DemoConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.RobotMap.CameraName;
import frc.robot.Constants.RobotConfig;
import frc.robot.Constants.LiftConstants.LiftPosition;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DemoDrive;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FollowPath;
import frc.robot.commands.IndependentDrive;
import frc.robot.commands.PointAndDrive;
import frc.robot.commands.SpinManually;
import frc.robot.commands.SyncOffsets;
import frc.robot.commands.TandemDrive;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Path;
import frc.robot.util.WingPoseEstimator;
import frc.robot.subsystems.InputGetter;
import frc.robot.subsystems.InputSender;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.PhotonVision;

/**
 * RobotContainer sets up subsystems, commands, and input bindings for the robot.
 * Handles initialization and autonomous command selection.
 */
public class RobotContainer {
	public final Swerve swerve;
    private InputGetter inputGetter;
    private PhotonVision photonVision;
	private WingPoseEstimator wingPoseEstimator;
	private Lift lift;
	private CommandXboxController temp = new CommandXboxController(0);

	public RobotContainer() {
		// temp.a().whileTrue(new IndependentDrive(new Swerve(0), () -> new Pose2d(1, 1, new Rotation2d(33)), () -> new Pose2d(1, 1, new Rotation2d(33)), () -> new Pose2d(1, 1, new Rotation2d(33))));
        inputGetter = new InputGetter();
		new InputSender();
        if(Constants.IS_MASTER) {
			swerve = new Swerve(0);
        } else {
			swerve = new Swerve(1);
		}
		wingPoseEstimator = new WingPoseEstimator();
		photonVision = new PhotonVision(swerve, wingPoseEstimator);
		lift = new Lift();
		// swerve.setDefaultCommand(new IndependentDrive(swerve, () -> new Pose2d(1, 1, new Rotation2d(43)), () -> new Pose2d(1, 1, new Rotation2d(43)), () -> new Pose2d(1, 1, new Rotation2d(43))));
		// new Trigger(() -> inputGetter.getBButton()).onTrue(new TandemDrive(swerve, inputGetter::getJoystickVelocity).ignoringDisable(true));
		// new Trigger(() -> inputGetter.getXButton()).onTrue(RobotConfig.reset());
		// //manually spin individual pods based on dpad input + right joystick input
		// new Trigger(() -> (inputGetter.getPOV() == -1)).onFalse(new SpinManually(swerve, () -> getPodToTest(), () -> inputGetter.getRightX()));
		
		// //point all pods at an angle based on right joystick input
		// new Trigger(inputGetter::getLeftBumper).whileTrue(new PointAndDrive(swerve, () -> new Translation2d(inputGetter.getRightX(), inputGetter.getRightY()), () -> inputGetter.getRightTriggerAxis()));
		
		// new Trigger(inputGetter::getStartButton).whileTrue(new InstantCommand(swerve::resetPods, swerve));
		// Pose2d wingApproximate = Constants.IS_MASTER ? DemoConstants.masterWingApproximate : DemoConstants.slaveWingApproximate;
		// new Trigger(inputGetter::getAButton).onTrue(new AutoDrive(swerve, () -> wingApproximate, true));
		// new Trigger(inputGetter::getBButton).onTrue(new AutoDrive(swerve, wingPoseEstimator::getEstimatedPose, false));
		new Trigger(inputGetter::getYButton).whileTrue(new DemoDrive(swerve, lift, wingPoseEstimator, inputGetter));
		
		new Trigger(inputGetter::getLeftBumper).whileTrue(new TandemDrive(swerve, inputGetter::getJoystickVelocity));
		new Trigger(inputGetter::getRightBumper).whileTrue(new IndependentDrive(swerve, () -> inputGetter.getLeftJoystick(), () -> inputGetter.getRightJoystick()));
		
		// new Trigger(inputGetter::getBButton).whileTrue(lift.setLiftState(LiftPosition.pickup)).onFalse(new InstantCommand(lift::stop, lift));
		// new Trigger(inputGetter::getXButton).whileTrue(lift.setLiftState(LiftPosition.place)).onFalse(new InstantCommand(lift::stop, lift));
		new Trigger(inputGetter::getBButton).whileTrue(new InstantCommand(() -> lift.setPosition(15), lift)).onFalse(new InstantCommand(lift::stop, lift));
		new Trigger(inputGetter::getXButton).whileTrue(new InstantCommand(() -> lift.setPosition(6), lift)).onFalse(new InstantCommand(lift::stop, lift));
		
		// new Trigger(inputGetter::getAButton).whileTrue(new RunCommand(() -> lift.setPower(-0.1), lift)).onFalse(new InstantCommand(lift::stop, lift));
		// new Trigger(inputGetter::getYButton).whileTrue(new RunCommand(() -> lift.setPower(0.1), lift)).onFalse(new InstantCommand(lift::stop, lift));
		
		new Trigger(inputGetter::getStartButton).whileTrue(new InstantCommand(lift::resetEncoder, lift));
		new Trigger(inputGetter::getAButton).whileTrue(new FollowPath(swerve, new Path(() -> swerve.getPose(), PathConstants.wayPoints, PathConstants.defaultSpeed, PathConstants.lookAhead, PathConstants.rotationalLookAhead), PathConstants.wayPoints.get(PathConstants.wayPoints.size()-1)));
		// swerve.setDefaultCommand(new DemoDrive(swerve, wingPoseEstimator, inputGetter));


		//drive bindings
		Supplier<Translation2d> driverRightJoystick = () -> new Translation2d(inputGetter.getRightX(),
				inputGetter.getRightY());
		Supplier<Translation2d> driverLeftJoystick = () -> new Translation2d(inputGetter.getLeftX(),
				inputGetter.getLeftY());
		// swerve.setDefaultCommand(
		// 		new DriveCommand(swerve, driverLeftJoystick, driverRightJoystick));



		// PID tuning/testing function. just sets FL pod to DPAD angle.
		// swerve.setDefaultCommand(new RunCommand(() -> swerve.getPods().get(0).setPodState(new SwerveModuleState(joystick.getLeftY(), Rotation2d.fromDegrees(joystick.getHID().getPOV()))), swerve));
	}
	
	private int getPodToTest() {
		switch (inputGetter.getPOV()) {
			case 0:
				return 0; //FL
			case 90:
				return 1; //FR
			case 180:
				return 3; //BR
			case 270:
				return 2; //BL
			default:
				return -1;
		}
		
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
