// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.sound.sampled.Line;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConfig;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.RobotMap.PodConfig;
import frc.robot.Constants.RobotConfig.SingleRobotConfig;
import frc.robot.util.LegacyPoseEstimator;
import frc.robot.util.NERDPoseEstimator;
import frc.robot.Constants;
import frc.robot.Robot;

/**a subsystem for a NERDSwerve robot */
public class Swerve extends SubsystemBase {
	private final Pigeon2 gyro;


	
	private final ArrayList<DrivePod> pods = new ArrayList<DrivePod>();

	private final NERDPoseEstimator poseEstimator;
	private final SwerveDriveKinematics drivetrainKinematics;

	public double targetAngle = 0;

	private final Field2d field2d = new Field2d();

	StructArrayPublisher<SwerveModuleState> SMSPublisher;
	StructPublisher<Pose2d> PosePublisher;
	StructPublisher<Pose2d> VisionPosePublisher;
	StructPublisher<Pose2d> OdoPosePublisher;
	StructPublisher<ChassisSpeeds> ChassisSpeedsPublisher;
	DoubleEntry azimuthkPSub, azimuthkISub, azimuthkDSub, azimuthkSSub, azimuthkVSub, azimuthkASub;


	
	public Swerve(int robot) {
		SingleRobotConfig config = RobotConfig.robotConfigs[robot];
		drivetrainKinematics = config.drivetrainKinematics;
		gyro = new Pigeon2(RobotConfig.pigeonID);

		for (int i = 0; i < RobotConfig.robotConfigs[robot].PodConfigs.length; i++) {
			PodConfig podConfig = config.PodConfigs[i];
			pods.add(
					new DrivePod(podConfig.encoderID, podConfig.azimuthID, podConfig.driveID, podConfig.encoderOffset, RobotConfig.azimuthInvert, 
					RobotConfig.azimuthAmpLimit, RobotConfig.azimuthRadiansPerMotorRotation, RobotConfig.azimuthBrake, RobotConfig.azimuthMotorRampRate, 
					podConfig.kP, podConfig.kI, podConfig.kD, podConfig.kS, podConfig.kV, podConfig.kA, 
					RobotConfig.azimuthMaxOutput, RobotConfig.azimuthDriveSpeedMultiplier, RobotConfig.driveInvert, 
					RobotConfig.driveAmpLimit, RobotConfig.driveBrake, RobotConfig.driveMotorRampRate));
		}


		// initialize odometry based on the pod positions
		poseEstimator = new NERDPoseEstimator(
				drivetrainKinematics,
				getGyro(),
				getModulePositions(),
				new Pose2d());


		// gyro.configAllSettings(new Pigeon2Configuration());

		// initialize telemetry publishers
		String tab = Constants.currentRobot.toString();

		SMSPublisher = NetworkTableInstance.getDefault().getTable(tab).getStructArrayTopic("ModuleStates", SwerveModuleState.struct)
				.publish();
		PosePublisher = NetworkTableInstance.getDefault().getTable(tab).getStructTopic("RobotPose", Pose2d.struct).publish(Constants.NTPubSub);
		VisionPosePublisher = NetworkTableInstance.getDefault().getTable(tab).getStructTopic("VisionPose", Pose2d.struct).publish();
		ChassisSpeedsPublisher = NetworkTableInstance.getDefault().getTable(tab).getStructTopic("ChassisSpeeds", ChassisSpeeds.struct)
				.publish();
		OdoPosePublisher = NetworkTableInstance.getDefault().getTable(tab).getStructTopic("OdoPose", Pose2d.struct).publish();

		// initialize PID subscribers
		if(Constants.tuningMode) {
			azimuthkPSub = NetworkTableInstance.getDefault().getTable(tab).getSubTable("PIDs").getDoubleTopic("kP").getEntry(RobotConfig.robotConfigs[robot].PodConfigs[0].kP);
			azimuthkISub = NetworkTableInstance.getDefault().getTable(tab).getSubTable("PIDs").getDoubleTopic("kI").getEntry(RobotConfig.robotConfigs[robot].PodConfigs[0].kI);
			azimuthkDSub = NetworkTableInstance.getDefault().getTable(tab).getSubTable("PIDs").getDoubleTopic("kD").getEntry(RobotConfig.robotConfigs[robot].PodConfigs[0].kD);

			azimuthkPSub.set(RobotConfig.robotConfigs[robot].PodConfigs[0].kP);
			azimuthkISub.set(RobotConfig.robotConfigs[robot].PodConfigs[0].kI);
			azimuthkDSub.set(RobotConfig.robotConfigs[robot].PodConfigs[0].kD);

			azimuthkDSub.getAtomic();
		}

	}

	public synchronized void updateOdo() {
		//update odometry
		poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyro(), getModulePositions());

	}

	@Override
	public void periodic() {

		updateOdo();
		

		//update telemetry
		field2d.setRobotPose(getPose());

		SMSPublisher.set(getModuleStates());
		// if (Constants.IS_MASTER) {
		PosePublisher.set(getPose());
		OdoPosePublisher.set(poseEstimator.getOdometry());

		// } else {
		// 	PosePublisher.set(new Pose2d());
		// }
		ChassisSpeedsPublisher.set(drivetrainKinematics.toChassisSpeeds(getModuleStates()));


		// update the numbers (if tuning mode is enabled)
		if (Constants.tuningMode) {
			pods.forEach(pod -> {
			pod.setPID(
					azimuthkPSub.get(),
					azimuthkISub.get(),
					azimuthkDSub.get());
		});
		}
	}

	/**
	 * gets the current robot chassis speeds
	 * 
	 * @return the current robot chassis speeds
	 */
	private ChassisSpeeds getCurrentRobotChassisSpeeds() {
		// get the current robot chassis speeds
		return drivetrainKinematics.toChassisSpeeds(getModuleStates());
	}

	/**
	 * gets the current robot pose
	 * 
	 * @return the current robot pose
	 */
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/**
	 * resets the odometry to the given pose
	 * 
	 * @param pose the new pose to reset to
	 */
	public void resetOdometry(Pose2d pose) {
		poseEstimator.resetPosition(new Rotation2d(), getModulePositions(), pose);
	}

	/**
	 * gets the current robot chassis speeds
	 * 
	 * @return the current robot chassis speeds
	 */
	public SwerveModulePosition[] getModulePositions() {
		return pods.stream()
				.map(DrivePod::getPodPosition)
				.toArray(SwerveModulePosition[]::new);
	}

	/**
	 * gets the current robot module states
	 * 
	 * @return the current robot module states
	 */
	public SwerveModuleState[] getModuleStates() {
		return pods.stream()
				.map(DrivePod::getState)
				.toArray(SwerveModuleState[]::new);
	}

	/**
	 * resets the pods and odometry zero
	 */
	public void resetPods() {
		resetGyro();
		pods.stream().forEach(DrivePod::resetPod);

		resetOdometry(new Pose2d());
	}

	/**
	 * sets all motor output to 0
	 */
	public void stop() {
		pods.stream().forEach(DrivePod::stop);
	}

	public Rotation2d getGyro() {
		return gyro.getRotation2d();
	}

	public void resetGyro() {
		gyro.setYaw(0);
	}

	/**
	 * sets the robot's chassis speeds based on the given chassis speeds, enables isTurning
	 * @param chassisSpeeds the desired chassis speeds (+x forward, +y left, +omega counter-clockwise)
	 */
	public void setRobotSpeeds(ChassisSpeeds chassisSpeeds) {
		setRobotSpeeds(chassisSpeeds, true);
	}

	/**
	 * sets the robot's chassis speeds based on the given chassis speeds
	 * @param chassisSpeeds the desired chassis speeds (+x forward, +y left, +omega counter-clockwise)
	 * @param enableIsTurning if true, the robot will not move if any pod is turning and will not start turning if any pod is still moving
	 */
	public void setRobotSpeeds(ChassisSpeeds chassisSpeeds, boolean enableIsTurning) {
		SwerveModuleState[] states = drivetrainKinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, RobotConfig.robotMaxSpeed);

		for (int i = 0; i < pods.size(); i++) {
			pods.get(i).setPodState(states[i], enableIsTurning);
		}
	}

	public ArrayList<DrivePod> getPods() {
		return pods;
	}

	/**
	 * Sanity-checks a vision pose before forwarding it to NERDPoseEstimator.
	 *
	 * <p>Only rejects obviously invalid poses (NaN/infinite coordinates). All
	 * distance-based outlier rejection and adaptive acceptance-radius logic lives
	 * inside {@link frc.robot.util.NERDPoseEstimator#addVisionMeasurement}, which
	 * has the context needed to make that decision correctly (current estimate,
	 * time since last accepted update, etc.).
	 *
	 * @param visionPose The vision-estimated robot pose to validate.
	 * @return {@code true} if the pose contains only finite values.
	 */
	private boolean isVisionMeasurementReasonable(Pose2d visionPose) {
		return Double.isFinite(visionPose.getX())
				&& Double.isFinite(visionPose.getY())
				&& Double.isFinite(visionPose.getRotation().getRadians());
	}

	/**
	 * accepts a vision measurement for pose estimation
	 * 
	 * @param visionPose estimated pose from the vision system
	 * @param stdDev   standard deviation representing uncertainty in the pose estimation
	 */

	public void addVisionMeasurement(Pose2d visionPose, double timestamp, double stdDev) {
		VisionPosePublisher.set(visionPose);


		if (!isVisionMeasurementReasonable(visionPose)) {
			// return;
		}
		

		if(DriverStation.isDisabled() && stdDev < 9) {
			// fully sync pose estimator to vision on disable as long as the vision update isnt poop
			poseEstimator.addVisionMeasurement(visionPose,
					timestamp,
					VecBuilder.fill(0.01, 0.01, 0.01));
		} else {
			// add some bias to odo on enable
			poseEstimator.addVisionMeasurement(visionPose,
					timestamp,
					// decreases vision confidence with distance+ambiguity
					VecBuilder.fill(stdDev, stdDev, stdDev));
		}


		// if(stdDev < 9) {
		// 	System.out.println("vision reject; high stddev");
		// } 

	}

	
	// /**
	//  * sets pose to this
	//  * 
	//  * @param pose estimated pose from the vision system
	//  */
	// public void resetPose(Pose2d pose) {
	// 	poseEstimator.resetPose(pose);
	// }
}
