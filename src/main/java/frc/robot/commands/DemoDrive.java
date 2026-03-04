
package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.DemoConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.RobotConfig;
import frc.robot.Constants.LiftConstants.LiftPosition;
import frc.robot.subsystems.InputGetter;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Path;
import frc.robot.util.WingPoseEstimator;

public class DemoDrive extends SequentialCommandGroup {
    BooleanSubscriber isOtherRobotFinished;
    

    Supplier<Integer> wingEstimateApproximateToUse;
    Supplier<Integer> wingVisionApproximateToUse;
    StructSubscriber<Pose2d> otherRobotPoseSub = NetworkTableInstance.getDefault().getTable(Constants.currentRobot.getOpposite().toString()).getStructTopic("RobotPose", Pose2d.struct).subscribe(new Pose2d());
    StructSubscriber<Pose2d> otherRobotWingPoseSub = NetworkTableInstance.getDefault().getTable("WingPoseEstimator").getStructTopic(Constants.currentRobot.getOpposite().toString() + " pose estimate", Pose2d.struct).subscribe(new Pose2d());

    /** Creates a new DemoDrive. */
    public DemoDrive(Swerve swerve, Lift lift, WingPoseEstimator wingPoseEstimator, InputGetter inputGetter) {
        isOtherRobotFinished = NetworkTableInstance.getDefault().getTable(Constants.currentRobot.getOpposite().toString()).getBooleanTopic("autodrive at target").subscribe(false);
        
        // have the master determine which wing position is closer
        if(Constants.IS_MASTER) {
            BooleanPublisher useMasterTagPublisher = NetworkTableInstance.getDefault().getTable("DemoMode").getBooleanTopic("use master tag").publish();
            wingEstimateApproximateToUse = () -> {
            if(Math.abs(swerve.getPose().minus(DemoConstants.wingApproximates[0]).getTranslation().getNorm()) < (Math.abs(swerve.getPose().minus(DemoConstants.wingApproximates[1]).getTranslation().getNorm()))) {
                useMasterTagPublisher.accept(false);
                return 0;
            } else {
                useMasterTagPublisher.accept(true);
                return 1;
            }};
        } else {
            BooleanSubscriber useMasterTagSubscriber = NetworkTableInstance.getDefault().getTable("DemoMode").getBooleanTopic("use master tag").subscribe(true);
            wingEstimateApproximateToUse = () -> useMasterTagSubscriber.get() ? 0 : 1;
        }

        
        addCommands(
            new InstantCommand(() -> resetPublishers()),
            new DriveToWing(swerve, otherRobotPoseSub, otherRobotWingPoseSub, wingPoseEstimator, DemoConstants.estimatedWingPosition, DemoConstants.wingRelativeFormationOffsets),
            // new AutoDrive(swerve, () -> DemoConstants.wingApproximates[wingEstimateApproximateToUse.get()], true), // follow path to approximate wing position
            // new InstantCommand(() -> chooseWingVisionApproximateToUse(swerve, wingPoseEstimator)),
            // new WaitUntilCommand(isOtherRobotFinished::get),
            // new WaitCommand(0.5),
            // new AutoDrive(swerve, () -> wingPoseEstimator.getEstimatedPose().plus(DemoConstants.wingRelativeFormationOffsets[wingVisionApproximateToUse.get()]), false), // PID to exact wing position
            new WaitUntilCommand(isOtherRobotFinished::get), // wait for other robot to finish
            // new WaitCommand(0.2),
            lift.setLiftState(LiftPosition.pickup).withTimeout(3), // lift up
            new SyncOffsets(swerve).withTimeout(1), // sync offsets (unnecessary)
            // new InstantCommand(RobotConfig::resetOffsetPositions), // sets ofsets to original
            // new TandemDrive(swerve, inputGetter::getJoystickVelocity).until(inputGetter::getRightBumper), // tandem drive with other robot manually
            new FollowPath(swerve, new Path(() -> swerve.getPose(), PathConstants.wayPoints, PathConstants.defaultSpeed, PathConstants.lookAhead, PathConstants.rotationalLookAhead), PathConstants.wayPoints.get(PathConstants.wayPoints.size()-1)) // follow path to station
        );
    }

    public void resetPublishers() {
        IntegerPublisher useMasterTagPublisher = NetworkTableInstance.getDefault().getTable("DemoMode").getIntegerTopic("use master tag post-vision").publish();
        useMasterTagPublisher.accept(-1);
    }

    public void chooseWingVisionApproximateToUse(Swerve swerve, WingPoseEstimator wingPoseEstimator) {
        IntegerSubscriber useMasterTagSubscriber = NetworkTableInstance.getDefault().getTable("DemoMode").getIntegerTopic("use master tag post-vision").subscribe(-1);
        if ((int)useMasterTagSubscriber.get() == -1) {
            IntegerPublisher useMasterTagPublisher = NetworkTableInstance.getDefault().getTable("DemoMode").getIntegerTopic("use master tag post-vision").publish();
                wingVisionApproximateToUse = () -> {
                if(Math.abs(swerve.getPose().minus(wingPoseEstimator.getEstimatedPose().plus(DemoConstants.wingRelativeFormationOffsets[0])).getTranslation().getNorm()) < Math.abs(swerve.getPose().minus(wingPoseEstimator.getEstimatedPose().plus(DemoConstants.wingRelativeFormationOffsets[1])).getTranslation().getNorm())) {
                    useMasterTagPublisher.accept(1);
                    return 0;
                } else {
                    useMasterTagPublisher.accept(0);
                    return 1;
                }};
        } else {
            wingVisionApproximateToUse = () -> ((int)useMasterTagSubscriber.get());
        }
    }

}