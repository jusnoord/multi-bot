//generic class to handle alll inputs from DS

package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanArraySubscriber;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.RobotConfig;;

/**
 * InputInterface handles all inputs from the driver station, both on master and slave robots.
 * Manages NetworkTables server/client setup and input publishing/subscribing.
 */
public class InputInterface {
	private static Inputs inputs;
	private static NetworkTable table = NetworkTableInstance.getDefault().getTable("Inputs");

	// server-side

	private static DoubleArrayPublisher sticksPublisher;
	private static BooleanArrayPublisher buttonsPublisher;
	private static BooleanPublisher isEnabledPublisher;
	private static DoublePublisher timeStampPublisher;
	private static StructPublisher<Pose2d> joystickVelocityPublisher;
	private static StructPublisher<Pose2d> masterOffsetPublisher;

	public static void initializeServer() {
		publishInputs();
		subscribeInputs();
	}

	private static void publishInputs() {
		sticksPublisher = table.getDoubleArrayTopic("sticks").publish(Constants.NTPubSub);

		buttonsPublisher = table.getBooleanArrayTopic("buttons").publish(Constants.NTPubSub);
		isEnabledPublisher = table.getBooleanTopic("isEnabled").publish(Constants.NTPubSub);
		timeStampPublisher = table.getDoubleTopic("timeStamp").publish(Constants.NTPubSub);
		joystickVelocityPublisher = table.getStructTopic("joystickVelocity", Pose2d.struct).publish(Constants.NTPubSub);
		// masterOffsetPublisher = table.getStructTopic("masterOffset", Pose2d.struct).publish();
	}

	public static void updateInputs(Pose2d masterOffset) {
		inputs = new Inputs(masterOffset);
}

	public static void updateInputs(XboxController controller, boolean isenabled, double timeStamp, Pose2d joystickVelocity) {
		inputs = new Inputs(controller, isenabled, timeStamp, joystickVelocity);

		sticksPublisher.set(new double[] {
				inputs.leftX, inputs.leftY, inputs.rightX, inputs.rightY, inputs.leftTrigger, inputs.rightTrigger, inputs.pov
		});
		buttonsPublisher.set(new boolean[] {
				inputs.aButton, inputs.bButton, inputs.xButton, inputs.yButton,
				inputs.leftBumper, inputs.rightBumper, inputs.startButton, inputs.backButton
		});
		isEnabledPublisher.set(inputs.isEnabled);
		timeStampPublisher.set(inputs.timeStamp);
		joystickVelocityPublisher.set(joystickVelocity);
		// masterOffsetPublisher.set(inputs.masterOffset);
	}

	// client-side

	private static DoubleArraySubscriber sticksSubscriber;
	private static BooleanArraySubscriber buttonsSubscriber;
	private static BooleanSubscriber isEnabledSubscriber;
	private static DoubleSubscriber timeStampSubscriber;
	private static StructSubscriber<Pose2d> joystickVelocitySubscriber;
	private static StructSubscriber<Pose2d> masterOffsetSubscriber;

	/** initializes the client-side inputs, should be called once at the start of the robot code on slave(s*/
	public static void initializeClient() {
		NetworkTableInstance.getDefault().stopServer(); // Close the server if this is a slave robot
		NetworkTableInstance.getDefault().startClient4("slave"); // if multiple slaves are implemented, this has to be keyed
		NetworkTableInstance.getDefault().setServer("roboRIO-9312-FRC.local");

		subscribeInputs();
	}

	private static void subscribeInputs() {
		sticksSubscriber = table.getDoubleArrayTopic("sticks").subscribe(new double[] { 0, 0, 0, 0, 0, 0, 0 }, Constants.NTPubSub);
		buttonsSubscriber = table.getBooleanArrayTopic("buttons")
				.subscribe(new boolean[] { false, false, false, false, false, false, false, false }, Constants.NTPubSub);
		isEnabledSubscriber = table.getBooleanTopic("isEnabled").subscribe(false, Constants.NTPubSub);
		timeStampSubscriber = table.getDoubleTopic("timeStamp").subscribe(0.0, Constants.NTPubSub);
		joystickVelocitySubscriber = table.getStructTopic("joystickVelocity", Pose2d.struct).subscribe(new Pose2d(), Constants.NTPubSub);
		masterOffsetSubscriber = table.getStructTopic("masterOffset", Pose2d.struct).subscribe(new Pose2d(), Constants.NTPubSub);
	}

	public static Inputs grabInputs() {
		return new Inputs(
				sticksSubscriber.get(),
				buttonsSubscriber.get(),
				isEnabledSubscriber.get(),
				timeStampSubscriber.get(),
				joystickVelocitySubscriber.get(), 
				masterOffsetSubscriber.get());
	}

	/**
	 * Inputs holds all input values passed from master to slave.
	 * Includes joystick axes, button states, and robot status.
	 */
	public static class Inputs {
		public double leftX;
		public double leftY;
		public double rightX;
		public double rightY;
		public double leftTrigger;
		public double rightTrigger;
		public int pov;
		public boolean aButton;
		public boolean bButton;
		public boolean xButton;
		public boolean yButton;
		public boolean leftBumper;
		public boolean rightBumper;
		public boolean startButton;
		public boolean backButton;
		public boolean isEnabled;
		public double timeStamp;
		public Pose2d joystickVelocity;
		// public Transform2d masterOffset = RobotConfig.offsetPositions[0];
		
		public Inputs (Pose2d masterOffset) {
			// this.masterOffset = masterOffset;
		}

		public Inputs(XboxController controller, boolean isenabled, double timeStamp, Pose2d joystickVelocity) {
			this.joystickVelocity = joystickVelocity;

			//double values
			this.isEnabled = isenabled;
			this.timeStamp = timeStamp;
			this.leftX = MathUtil.applyDeadband(controller.getLeftX(), JoystickConstants.deadband);
			this.leftY = MathUtil.applyDeadband(controller.getLeftY(), JoystickConstants.deadband);
			this.rightX = MathUtil.applyDeadband(controller.getRightX(), JoystickConstants.deadband);
			this.rightY = MathUtil.applyDeadband(controller.getRightY(), JoystickConstants.deadband);
			this.leftTrigger = controller.getLeftTriggerAxis();
			this.rightTrigger = controller.getRightTriggerAxis();
			this.pov = controller.getPOV();
			
			//boolean values
			this.aButton = controller.getAButton();
			this.bButton = controller.getBButton();
			this.xButton = controller.getXButton();
			this.yButton = controller.getYButton();
			this.leftBumper = controller.getLeftBumperButton();
			this.rightBumper = controller.getRightBumperButton();
			this.startButton = controller.getStartButton();
			this.backButton = controller.getBackButton();
		}
		public Inputs(XboxController controller, boolean isenabled, double timeStamp, Pose2d joystickVelocity, Pose2d masterOffset) {
			this.joystickVelocity = joystickVelocity;
			// this.masterOffset = masterOffset;

			//double values
			this.isEnabled = isenabled;
			this.timeStamp = timeStamp;
			this.leftX = MathUtil.applyDeadband(controller.getLeftX(), JoystickConstants.deadband);
			this.leftY = MathUtil.applyDeadband(controller.getLeftY(), JoystickConstants.deadband);
			this.rightX = MathUtil.applyDeadband(controller.getRightX(), JoystickConstants.deadband);
			this.rightY = MathUtil.applyDeadband(controller.getRightY(), JoystickConstants.deadband);
			this.leftTrigger = controller.getLeftTriggerAxis();
			this.rightTrigger = controller.getRightTriggerAxis();
			this.pov = controller.getPOV();
			
			//boolean values
			this.aButton = controller.getAButton();
			this.bButton = controller.getBButton();
			this.xButton = controller.getXButton();
			this.yButton = controller.getYButton();
			this.leftBumper = controller.getLeftBumperButton();
			this.rightBumper = controller.getRightBumperButton();
			this.startButton = controller.getStartButton();
			this.backButton = controller.getBackButton();
		}

		public Inputs(double[] sticks, boolean[] buttons, boolean isEnabled, double timeStamp, Pose2d joystickVelocity, Pose2d masterOffset) {
			this.joystickVelocity = joystickVelocity;
			// this.masterOffset = masterOffset;
			//double values
			this.leftX = MathUtil.applyDeadband(sticks[0], JoystickConstants.deadband);
			this.leftY = MathUtil.applyDeadband(sticks[1], JoystickConstants.deadband);
			this.rightX = MathUtil.applyDeadband(sticks[2], JoystickConstants.deadband);
			this.rightY = MathUtil.applyDeadband(sticks[3], JoystickConstants.deadband);
			this.leftTrigger = sticks[4];
			this.rightTrigger = sticks[5];
			this.pov = (int) sticks[6];

			//boolean values
			this.aButton = buttons[0];
			this.bButton = buttons[1];
			this.xButton = buttons[2];
			this.yButton = buttons[3];
			this.leftBumper = buttons[4];
			this.rightBumper = buttons[5];
			this.startButton = buttons[6];
			this.backButton = buttons[7];
			this.isEnabled = isEnabled;
			this.timeStamp = timeStamp;
		}
	}
}
