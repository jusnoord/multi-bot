// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.RobotMap.PodConfig;

/**
 * Constants class holds all configuration values and constants for the robot.
 * Includes robot type, CAN IDs, kinematics, PID gains, and hardware settings.
 * Organized into nested classes for logical grouping.
 */
public final class Constants {
	public static final boolean tuningMode = true; // if true, the robot will use the dashboard to get values for PIDs

    //grab the master/slave from a file on the robot. This will determine whether it's running the shuffleboard server.
    // public static final Path MASTER_PATH = Paths.get("/home/lvuser/master");
	public static final File MASTER_FILE = new File("/is_master");
    public static final boolean IS_MASTER = MASTER_FILE.exists(); // MASTER_PATH.toFile().exists();
	//TODO: phase out if statements and replace them with switches w/ this enum
	public static enum RobotType {
		master, slave;

		public RobotType getOpposite() {
			return this == master ? slave : master;
		}
	} 

	public static RobotType currentRobot = IS_MASTER ? RobotType.master : RobotType.slave;

	public static final PubSubOption[] NTPubSub = {PubSubOption.sendAll(true), PubSubOption.periodic(0.005)};

	/**
	 * RobotMap contains hardware mapping and configuration for robot components.
	 * Includes pod configurations, camera names, and kinematics.
	 */
	public final class RobotMap {
		//lift
		public static final int liftMotorID = 30;
		public static final int canRangeID = 31;
		public static final boolean liftInvert = false;
		public static final double liftStatorLimit = 10; // shrug
		public static final boolean liftBrake = false;
		public static final double liftEncoderToMechanismRatio = 50/(Inches.of(1.25).in(Centimeters) * Math.PI); //50:1 gearbox, 1.25" drum
		public static final double liftRotorToEncoderRatio = 1;

		// lift PID gains
		public static final double liftkP = 1;
		public static final double liftkI = 0.0;
		public static final double liftkD = 0.0;


		public static enum CameraType {
			front, back, top;
			
			public final CameraName getCameraName() {
				if (IS_MASTER) {
					switch (this) {
						case front:
							return CameraName.masterFront;
						case back:
							return CameraName.masterBack;
						case top:
							return CameraName.masterTop;
						default:
							return CameraName.masterFront;
					}
				} else {
					switch (this) {
						case front:
							return CameraName.slaveFront;
						case back:
							return CameraName.slaveBack;
						case top:
							return CameraName.slaveTop;
						default:
							return CameraName.slaveFront;
					}
				}
			}

			public final Transform3d getCameraPose() {
				switch (this) {
					case front:
						return VisionConstants.frontCameraPose;
					case back:
						return VisionConstants.backCameraPose;
					case top:
						return VisionConstants.topCameraPose;
					default:
						return VisionConstants.frontCameraPose;
				}
			}
		}
		// Camera IDs. this is for individual camera-threads, but there's only one so its fine
		public static enum CameraName {
			slaveFront, masterFront, masterBack, masterTop, slaveBack, slaveTop
		}



		/**
		 * PodConfig holds configuration for a single swerve pod.
		 * Includes motor IDs, encoder offsets, and position.
		 */
		/**
		 * PodConfig holds configuration for a single robot pod.
		 */
        public static final class PodConfig {
			public final int azimuthID;
			public final int driveID;
			public final int encoderID;
			public final double encoderOffset;
			public final Translation2d position;

			public static final boolean motorsBrake = true;

			public final int ampLimit;
			public final double maxOutput;
			public final double rampRate;

			public final double kP;
			public final double kI;
			public final double kD;
			public final double kS;
			public final double kV;
			public final double kA;

			public PodConfig(int azimuthID, int driveID, int encoderID, double encoderOffset, Translation2d podPosition) {
				this.azimuthID = azimuthID;
				this.driveID = driveID;
				this.encoderID = encoderID;
				this.encoderOffset = encoderOffset;
				this.position = podPosition;
				kP = 1;
				kI = 0.0;
				kD = 0.2;
				kS = 0;
				kV = 0;
				kA = 0;
				ampLimit = 80;
				maxOutput = 1;
				rampRate = 0.2;
			}


			public PodConfig(PodConfig podConfig, double kP, double kI, double kD, double kS, double kV, double kA) {
				this.azimuthID = podConfig.azimuthID;
				this.driveID = podConfig.driveID;
				this.encoderID = podConfig.encoderID;
				this.encoderOffset = podConfig.encoderOffset;
				this.position = podConfig.position;
				this.kP = kP;
				this.kI = kI;
				this.kD = kD;
				this.kS = kS; 
				this.kV = kV;
				this.kA = kA;
				ampLimit = podConfig.ampLimit;
				maxOutput = podConfig.maxOutput;
				rampRate = podConfig.rampRate;
			}

		}
	}

	public static final class PathConstants {
		public static final double defaultSpeed = 0.1; // m/s
		public static final double lookAhead = 0.08; // meters
		public static final Rotation2d rotationalLookAhead = Rotation2d.fromDegrees(25); 
		public static final List<Pose2d> wayPoints = new ArrayList<Pose2d>()
		{
			{
				add(new Pose2d(1.75, 1.9, Rotation2d.fromDegrees(0))); 
				add(new Pose2d(1.3, 1.45, Rotation2d.fromDegrees(90))); 
				add(new Pose2d(1.5, 1.1, Rotation2d.fromDegrees(180)));
				add(new Pose2d(1.0, 0.8, Rotation2d.fromDegrees(180)));
				add(new Pose2d(2.2, 0.8, Rotation2d.fromDegrees(180)));
			}
		};
	}

	/**
	 * RobotConstants contains robot-wide constants for control and hardware.
	 * Includes speed limits, PID gains, and camera/vision offsets.
	 */
	public final class RobotConstants {
		public static final double robotMaxLinearSpeed = 1; // meters per second
		public static final double robotMaxRotationalSpeed = 2.0; // meters per second
		public static final double motorMaxOutput = 1.0; // max output of the motors

		//PID gains for tandem drive controller
		public static final double tandemkP = 0.3;
		public static final double tandemkI = 0.0;
		public static final double tandemkD = 0.0;

		public static final double tandemkP_angle = 1.4;
		public static final double tandemkI_angle = 0.0;
		public static final double tandemkD_angle = 0.0;

		public static final double tandemAngleTolerance = 0.06; // radians
		public static final double tandemPositionTolerance = 0.06; // meters

		//autodrive gains
		public static final double autoDrivekP = 0.5;
		public static final double autoDrivekI = 0.08;
		public static final double autoDrivekD = 0.05;

		public static final double autoDrivekP_angle = 0.7;
		public static final double autoDrivekI_angle = 0.0;
		public static final double autoDrivekD_angle = 0.0;

		public static final double highAutoDriveAngleTolerance = 0.2; // radians
		public static final double highAutoDrivePositionTolerance = 0.05; // meters
		public static final double lowAutoDriveAngleTolerance = 0.02; // radians
		public static final double lowAutoDrivePositionTolerance = 0.01; // meters

		public static final double tandemTranslation_deadband = 0.008;
		public static final double tandemAngle_deadband = 0.005;

		public static final double maxAutoDriveSpeedMetersPerSecond = 0.1; // m/s
		public static final double maxAutoDriveAngularSpeedRadiansPerSecond = 0.2; // rad/s


		//follow path angle gains
		public static final double pathkP_angle = 0.9;
		public static final double pathkI_angle = 0.0;
		public static final double pathkD_angle = 0.0;

		//velocity PID gains
		public static final double velocitykP = 0.5;
		public static final double velocitykI = 0.0;
		public static final double velocitykD = 0.0;

		public static final double velocitykP_angle = 0.4;
		public static final double velocitykI_angle = 0.0;
		public static final double velocitykD_angle = 0.0;


	}


	public final class VisionConstants {
		//forward, left, height; roll, pitch, yaw
		public static final Transform2d tagPose = new Transform2d(new Translation2d(0.197, 0), new Rotation2d()); // meters, distance from the center of the master robot to the tag
		public static final Transform3d frontCameraPose = new Transform3d(new Translation3d(0.254, 0, 0),new Rotation3d());
		public static final Transform3d backCameraPose = new Transform3d(new Translation3d(-0.118, 0, 0),new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180)));
		public static final Transform3d topCameraPose = new Transform3d(new Translation3d(0.229, 0, 0),new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-90), Units.degreesToRadians(180)));
		// public static final Transform3d cameraPose = new Transform3d(new Translation3d(0.0, 0, 0),new Rotation3d());
		//TODO: fix these
		public static final ArrayList<Integer> RobotTagIDs = new ArrayList<>(Arrays.asList(17, 18, 19, 20));
		public static final ArrayList<Integer> WingTagIDs = new ArrayList<>(Arrays.asList(1, 2));
		public static final ArrayList<Integer> GlobalTagIDs = new ArrayList<>(Arrays.asList(5,6,7,8));

		public static final Map<Integer, Transform2d> tagPoses = new HashMap<>() {{
			// testing tag
			put(5, new Transform2d(0, 0, Rotation2d.fromDegrees(0)));

			//robot-relative tags
			put(17, new Transform2d(0, -0.116, Rotation2d.fromDegrees(-35)));
			put(18, new Transform2d(0, -0.116, Rotation2d.fromDegrees(-35)));
			put(19, new Transform2d(0, 0.116, Rotation2d.fromDegrees(35)));
			put(20, new Transform2d(0, 0.116, Rotation2d.fromDegrees(35)));

			//field-relative tags
			put(6, new Transform2d(0, Units.feetToMeters(2), Rotation2d.fromDegrees(0)));
			put(7, new Transform2d(0, Units.feetToMeters(6), Rotation2d.fromDegrees(0)));
			put(8, new Transform2d(0, Units.feetToMeters(10), Rotation2d.fromDegrees(0)));
									// x could be negated here idk but I have high confidence that the axes are not swapped and the y and angles are correct
			//wing tags
			put(1, new Transform2d(0, -Units.inchesToMeters(16.75/2), Rotation2d.fromDegrees(0))); 
			put(2, new Transform2d(0, Units.inchesToMeters(16.75/2), Rotation2d.fromDegrees(0))); // (2.77, 0.99)
									// axes could be swapped here idk since its upside down
		}};
	}

	public final class DemoConstants {
		public static final Pose2d estimatedWingPosition = new Pose2d(1.76, 2.5, Rotation2d.fromDegrees(90));
		
		// public static final Pose2d stationPosition = PathConstants.wayPoints.getLast();
		
		public static final Transform2d[] wingRelativeFormationOffsets = {
			new Transform2d(0,  -0.5, Rotation2d.fromDegrees(90)), 
			new Transform2d(0,  0.5, Rotation2d.fromDegrees(-90))
		};
		public static final Pose2d wingApproximates[] = {estimatedWingPosition.transformBy(wingRelativeFormationOffsets[0]),estimatedWingPosition.transformBy(wingRelativeFormationOffsets[1])};
		
	}

	/**
	 * JoystickConstants contains configuration for joystick input.
	 * Includes port numbers and deadband values.
	 */
	public final class JoystickConstants {
		public static final int driverPort = 0; // port of the driver controller

		public static final double deadband = 0.05; // deadband for the joysticks
		public static final double triggerDeadband = 0.05; // deadband for the triggers
	}

	/**
	 * SimConstants contains simulation-specific constants for robot physics.
	 */
	public final class SimConstants {
		public static final double inertia = 0.0605; // kg*m^2
		public static final double mass = 50; // kg, approximate mass of the robot
		public static final double wheelRadius = 0.3; // meters
		public static final double trackWidth = 0.5; // meters, distance between left and right wheels
		public static final double gearRatio = 0.02; // gear ratio of the drivetrain

	}

	/**
	 * RobotConfig contains configuration for the robot's drive system and pods.
	 * Includes pod configs, kinematics, and reset logic.
	 */
	public class RobotConfig {
		public static final double driveMetersPerMotorRotation = 8.143/(Units.inchesToMeters(4)*Math.PI);//1/(Units.inchesToMeters(2) * Math.PI / 1.36); //Wheel Diameter M * PI / Enc Count Per Rev / Gear Ratio ((inverted))
        public static final double azimuthDriveSpeedMultiplier = 1.0/3.571;
        public static final double azimuthRadiansPerMotorRotation = 1/21.4286;

        public static final int pigeonID = 25;
        public static final PIDController gyroPID = new PIDController(0.046, 0d, 0.001);



		private static final double wheelBase = Units.inchesToMeters(19.75);
		private static final double trackWidth = Units.inchesToMeters(19.75);
		public static final Transform2d[] offsetPositions = {new Transform2d(new Translation2d(-1d, 0.0), new Rotation2d()), new Transform2d(new Translation2d(1d, 0.0), new Rotation2d(Math.PI))}; // default center of rotation of robot
		public static Command reset() {
			return new InstantCommand(() -> resetOffsetPositions());
		}
		public static void resetOffsetPositions() {
			offsetPositions[0] = new Transform2d(new Translation2d(-1d, 0.0), new Rotation2d());
			offsetPositions[1] = new Transform2d(new Translation2d(1d, 0.0), new Rotation2d(Math.PI));
		}

		/**
		 * SingleRobotConfig holds configuration for a robot's set of pods and kinematics.
		 */
		public static final class SingleRobotConfig {
			public PodConfig[] PodConfigs;
			public SwerveDriveKinematics drivetrainKinematics;
			public SingleRobotConfig(PodConfig[] podConfigs) {
				this.PodConfigs = podConfigs;
				this.drivetrainKinematics = new SwerveDriveKinematics(
					java.util.Arrays.stream(podConfigs)
						.map(pod -> pod.position)
						.toArray(Translation2d[]::new)
				);
			}
			public SingleRobotConfig(PodConfig[] podConfigs, double kP, double kI, double kD, double kS, double kV, double kA) {
				this.PodConfigs = podConfigs;
				for (int i = 0; i < podConfigs.length; i++) {
					podConfigs[i] = new PodConfig(podConfigs[i], kP, kI, kD, kS, kV, kA);
				}
				this.drivetrainKinematics = new SwerveDriveKinematics(
					java.util.Arrays.stream(podConfigs)
						.map(pod -> pod.position)
						.toArray(Translation2d[]::new)
				);
			}
		}
		public static final SingleRobotConfig[] robotConfigs = new SingleRobotConfig[] { 
			new SingleRobotConfig(new PodConfig[] { // first (red) robot
				new PodConfig(8, 4, 24, 0.3184, new Translation2d(wheelBase / 2, -trackWidth / 2)), // BL
				new PodConfig(5, 9, 22, 0.7172, new Translation2d(-wheelBase / 2, trackWidth / 2)), // FR
				new PodConfig(10, 6, 21, -0.5950, new Translation2d(wheelBase / 2, trackWidth / 2)), // BR
				new PodConfig(7, 11, 23, 0.1038, new Translation2d(-wheelBase / 2, -trackWidth / 2)) // FL - +
			}, 50.0, 0.0, 0.5, 0.1, 2.66, 0.0),
			new SingleRobotConfig(new PodConfig[] { // second (green) robot
				new PodConfig(8, 4, 24, 0.0591, new Translation2d(wheelBase / 2, -trackWidth / 2)), // BL
				new PodConfig(5, 9, 22, 0.7627, new Translation2d(-wheelBase / 2, trackWidth / 2)), // FR
				new PodConfig(10, 6, 21, -0.6006, new Translation2d(wheelBase / 2, trackWidth / 2)), // BR
				new PodConfig(7, 11, 23, 0.3977, new Translation2d(-wheelBase / 2, -trackWidth / 2)) // FL - +
			}, 50.0, 0.0, 0.5, 0.1, 2.66, 0.0)
		};


        public static final double robotMaxSpeed = 12; // joystick multiplier in meters per second
        public static final double formationMaxRotationalSpeed = 0.6; //maximum rotational speed of the formation in rad/s


        // Azimuth Settings
        public static final boolean azimuthBrake = true;

        public static final int azimuthAmpLimit = 80;
        public static final double azimuthMaxOutput = 1;



        public static final double azimuthMotorRampRate = 0.0;
		public static final boolean encoderInvert = true;
		public static final boolean azimuthInvert = true; 

        // Drive Settings
        public static final double podMaxSpeed = 1;

        public static final boolean driveBrake = true;

        public static final int driveAmpLimit = 80;
        public static final int boostDriveLimit = 90;
        public static final double driveMotorRampRate = 0.0;

		public static final boolean driveInvert = false; 
	}

	public final class LiftConstants {
		public static final double liftMaxHeight = 30; // centimeters
		public static final double canRangeOffset = 0.1; // what the canrange reads at lift-zero
		public static final double positionTolerance = 1; // centimeters
		public static final double maxOutput = 2.4; //volts, 20% max output

		public enum LiftPosition { //meters
			ground(1), pickup(28), place(6);
 			public final double height;
			private LiftPosition(double height) {
				this.height = height;
			}
		}
	}
}
