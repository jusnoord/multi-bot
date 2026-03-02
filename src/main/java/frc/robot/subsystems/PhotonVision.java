// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import org.ejml.data.SingularMatrixException;
import org.opencv.core.Mat;
import org.opencv.video.KalmanFilter;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOptions;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedObject;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotType;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.RobotMap.CameraName;
import frc.robot.Constants.RobotMap.CameraType;
import frc.robot.util.Triplet;
import frc.robot.util.Tuple;
import frc.robot.util.WingPoseEstimator;
import frc.robot.util.TimestampedVisionUpdate;
import frc.robot.util.TimestampedVisionUpdateStruct;

/**
 * PhotonVision subsystem constructs a Photon Vision camera and runs multi-threaded pose estimation.
 * Handles vision processing, pose updates, and communication with drivetrain.
 */
public class PhotonVision extends SubsystemBase {
    public static Pose2d closestOtherPose = new Pose2d();
    public static Pose2d closestCurrentPose = new Pose2d();

    List<TimestampedObject<Pose2d>> timestampedCurrentPoses = new ArrayList<>(); // pull from NT depending on is_master
    List<TimestampedObject<Pose2d>> timestampedOtherPoses = new ArrayList<>(); // pull from NT depending on is_master

    private StructSubscriber<Pose2d> otherPoseSubscriber; // TODO: balance the NT publishing between master and slave
    private StructSubscriber<Pose2d> currentPoseSubscriber; // TODO: balance the NT publishing between master and slave
    private StructPublisher<Pose2d> posePublisher; // TODO: balance the NT publishing between master and slave
    private StructPublisher<Pose2d> globalDisplacementPublisher; // TODO: balance the NT publishing between master and slave

    private StructPublisher<Pose2d> slaveVisionUpdatePublisher;
    private StructPublisher<Pose2d> masterVisionUpdatePublisher;


    private WingPoseEstimator wingPoseEstimator;
    private StructPublisher<Pose2d> wingPosePublisher; 

    private StructPublisher<TimestampedVisionUpdate> updateRequestPublisher; // pose2d
    private StructSubscriber<TimestampedVisionUpdate> updateRequestSubscriber; // pose2d
    
    private StructPublisher<TimestampedVisionUpdate> offsetPublisher; // transform
    private StructSubscriber<TimestampedVisionUpdate> offsetSubscriber;

    private double timeOfLastLocalUpdate;
    // public Pose2d pose = new Pose2d();

    // public KalmanFilter wingEstimator = new KalmanFilter(3, 3);

    private Swerve drivetrain;

    private CameraThread frontCamThread, backCamThread, topCamThread;

    public PhotonVision(Swerve drivetrain, WingPoseEstimator wingPoseEstimator) {
        this.drivetrain = drivetrain;

        frontCamThread = new CameraThread(CameraType.front.getCameraName(), CameraType.front.getCameraPose());
        backCamThread = new CameraThread(CameraType.back.getCameraName(), CameraType.back.getCameraPose());
        topCamThread = new CameraThread(CameraType.top.getCameraName(), CameraType.top.getCameraPose());
        frontCamThread.start();
        backCamThread.start();
        topCamThread.start();

        this.wingPoseEstimator = wingPoseEstimator;



        posePublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(Constants.currentRobot.toString()).getStructTopic("pose for localization only", Pose2d.struct).publish(Constants.NTPubSub);
        globalDisplacementPublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(Constants.currentRobot.toString()).getStructTopic("global displacement publisher", Pose2d.struct).publish(Constants.NTPubSub);
        // poseSubscriber = NetworkTableInstance.getDefault().getTable(Constants.currentRobot.getOpposite().toString()).getStructTopic("RobotPose", Pose2d.struct).subscribe(new Pose2d());

        otherPoseSubscriber = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(Constants.currentRobot.getOpposite().toString()).getStructTopic("pose for localization only", Pose2d.struct).subscribe(new Pose2d(), Constants.NTPubSub);
        currentPoseSubscriber = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(Constants.currentRobot.toString()).getStructTopic("pose for localization only", Pose2d.struct).subscribe(new Pose2d(), Constants.NTPubSub);
        
        updateRequestSubscriber = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(Constants.currentRobot.getOpposite().toString()).getStructTopic("update requests", TimestampedVisionUpdate.struct).subscribe(new TimestampedVisionUpdate(), Constants.NTPubSub);
        
        updateRequestPublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(Constants.currentRobot.toString()).getStructTopic("update requests", TimestampedVisionUpdate.struct).publish();
        wingPosePublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(Constants.currentRobot.toString()).getStructTopic("wing pose", Pose2d.struct).publish();
        
        if (!Constants.IS_MASTER) {
            offsetSubscriber = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(Constants.RobotType.slave.toString()).getStructTopic("offsets", TimestampedVisionUpdate.struct).subscribe(new TimestampedVisionUpdate());

            slaveVisionUpdatePublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(Constants.RobotType.slave.toString()).getStructTopic("vision updates", Pose2d.struct).publish();
            masterVisionUpdatePublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(Constants.RobotType.master.toString()).getStructTopic("vision updates", Pose2d.struct).publish();
        
        } else {
            offsetPublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(Constants.RobotType.slave.toString()).getStructTopic("offsets", TimestampedVisionUpdate.struct).publish();
        }
    }

    /**
     * Switch the pipeline of a camera
     *
     * @param camera   Camera to switch the pipeline of
     * @param pipeline Pipeline to switch to
     */
    public void switchPipelines(PhotonCamera camera, int pipeline) {
        camera.setPipelineIndex(pipeline);

        /*
         * if pipeline is switched out of 3d mode, the camera's thread should be paused, then resumed when switched back
         * this is to prevent the camera from trying to update the pose when it's unable to
         * error handling should catch this, but it's better to be safe than sorry
         *
         * threads should be paused with Thread.wait() and resumed with Thread.notify()
         */
    }

    /** 
     * @param timestamp time in seconds
     * @return master absolute position at time timestamp
     */
    Pose2d getMasterPosition(Double timestamp) {
        return (Constants.IS_MASTER) ? getCurrentRobotPosition(timestamp) : getOtherRobotPosition(timestamp);
    }

    /** 
     * @param timestamp time in seconds
     * @return master absolute position at time timestamp
     */
    Pose2d getSlavePosition(Double timestamp) {
        return (!Constants.IS_MASTER) ? getCurrentRobotPosition(timestamp) : getOtherRobotPosition(timestamp);
    }

    /** 
     * @param timestamp time in seconds
     * @return master absolute position at time timestamp
     */
    Pose2d getCurrentRobotPosition(Double timestamp) {
        // if(drivetrain.getPose() == null) {
        //     System.out.println("[PhotonVision]: WARNING: drivetrain pose is null");
        //     return new Pose2d();
        // }
        
        // return drivetrain.getPose();



        timestampedCurrentPoses.addAll(List.of(currentPoseSubscriber.readQueue()));

        if(timestampedCurrentPoses.size() == 0) {
            System.out.println("[PhotonVision]: WARNING: no other robot poses available");
            return new Pose2d();
        }

        //trim timestampedMasterPoses to 10 values (for efficiency)
        if (timestampedCurrentPoses.size() > 10) {
            timestampedCurrentPoses = timestampedCurrentPoses.subList(timestampedCurrentPoses.size() - 10, timestampedCurrentPoses.size());
        }

        //find the master pose with the closest timestamp to the camera's timestamp
        //NT timestamps are measured in microseconds, PhotonVision timestamps are seconds. Mulitiply by one million to convert/
        double visionTimeStampMicroSeconds = timestamp * 1000000d;
        closestCurrentPose = timestampedCurrentPoses.stream()
            .min((a, b) -> Double.compare(Math.abs(a.serverTime - visionTimeStampMicroSeconds), Math.abs(b.serverTime - visionTimeStampMicroSeconds)))
            .map(pose -> pose.value)
            .orElse(null);
        
        return closestCurrentPose;
    }

    /** 
     * @param timestamp time in seconds
     * @return master absolute position at time timestamp
     */
    Pose2d getOtherRobotPosition(Double timestamp) {
        timestampedOtherPoses.addAll(List.of(otherPoseSubscriber.readQueue()));

        if(timestampedOtherPoses.size() == 0) {
            System.out.println("[PhotonVision]: WARNING: no other robot poses available");
            return new Pose2d();
        }

        //trim timestampedMasterPoses to 10 values (for efficiency)
        if (timestampedOtherPoses.size() > 10) {
            timestampedOtherPoses = timestampedOtherPoses.subList(timestampedOtherPoses.size() - 10, timestampedOtherPoses.size());
        }

        //find the master pose with the closest timestamp to the camera's timestamp
        //NT timestamps are measured in microseconds, PhotonVision timestamps are seconds. Mulitiply by one million to convert/
        double visionTimeStampMicroSeconds = timestamp * 1000000d;
        closestOtherPose = timestampedOtherPoses.stream()
            .min((a, b) -> Double.compare(Math.abs(a.serverTime - visionTimeStampMicroSeconds), Math.abs(b.serverTime - visionTimeStampMicroSeconds)))
            .map(pose -> pose.value)
            .orElse(null);
        
        return closestOtherPose;
    }
    
    /** 
     * @param pose absolute pose of robot to inject into kalman filter
     * @param timestamp time of pose update, in seconds 
     * @param stdDev proportional measure of uncertainty, usually just distance in meters
     * this function will additionally update the robot-centric wing estimator to match
     */
    synchronized void updateCurrentRobot(TimestampedVisionUpdate update) {
        drivetrain.addVisionMeasurement(new Pose2d(update.translation, update.rotation), update.timestamp, update.stdDev);
    }

    /** 
     * @param pose absolute pose of robot to inject into kalman filter
     * @param timestamp time of pose update, in seconds 
     * @param stdDev proportional measure of uncertainty, usually just distance in meters
     * this function will additionally update the robot-centric wing estimator to match
     */
    void updateOtherRobot(TimestampedVisionUpdate update) {
        updateRequestPublisher.set(update);
    }

    /** 
     * @param pose absolute pose of wing to inject into kalman filter
     * @param timestamp time of pose update, in seconds (does not matter because wing does not have odometry)
     * @param stdDev proportional measure of uncertainty, usually just distance in meters
     * this function will additionally update the robot-centric wing estimator to match
     */
    void updateWingEstimate(TimestampedVisionUpdate update) {
        wingPoseEstimator.addVisionMeasurement(new Pose2d(update.translation, update.rotation));
        // TODO add timestamp not important
    }

    Pose2d getWingEstimate() {
        return wingPoseEstimator.getEstimatedPose();
    }

    /**
     * updates only the wing pose relative to the current robot
     * requires current pose and robot to wing offset
     * 
     * Notes:
     * each robot has an independent wing pose estimator 
     * used for wing-specific tags
     * @param update measured displacement between robot and wing along with timestamp of measurement
     */
    private synchronized void updateWingVision(TimestampedVisionUpdate update, int TagID) {
        Pose2d currentRobotPose = getCurrentRobotPosition(update.timestamp);
        Pose2d wingPose = currentRobotPose.plus(new Transform2d(update.translation, update.rotation).inverse()).plus(getTagPose(TagID).inverse());
        wingPosePublisher.accept(wingPose);

        updateWingEstimate(new TimestampedVisionUpdate(wingPose, update.timestamp, update.stdDev));
    }


    /** 
     * updates both the master and slave equally in opposite directions
     * updates masterWing and slaveWing pose to follow
     * requires master pose, slave pose, slave/master new pose
     * 
     * Notes:
     * calculated exclusively on master and sent to slave
     * used for offsets between robots
     * @param update measured displacement between robots along with timestamp of measurement where the difference is master-relative
     */
    // private synchronized void updateLocalVision(TimestampedVisionUpdate update, RobotType caller) {
    //     if (Constants.IS_MASTER) {
    //         // update pose estimates for both master and slave
    //         double timestamp = update.timestamp;
    //         double stdDev = update.stdDev;
    //         Rotation2d rotation = update.rotation.unaryMinus();
    //         Translation2d translation = update.translation.unaryMinus().rotateBy(rotation);

    //         Pose2d masterPosition = getMasterPosition(timestamp);
    //         Pose2d slavePosition = getSlavePosition(timestamp);

    //         Rotation2d averageHeading = masterPosition.getRotation().plus(slavePosition.getRotation()).times(0.5);

    //         // Rotation2d newMasterRotation = masterPosition.getRotation();
    //         Rotation2d newMasterRotation = averageHeading.plus(rotation.times(0.5));
    //         // Rotation2d newSlaveRotation = slavePosition.getRotation();
    //         Rotation2d newSlaveRotation = averageHeading.plus(rotation.times(-0.5));

    //         Translation2d centerOfFormation = masterPosition.getTranslation().plus(slavePosition.getTranslation()).times(0.5);

    //         Translation2d displacementGlobalFrame = translation.rotateBy(newMasterRotation);

    //         Translation2d newSlaveTranslation = displacementGlobalFrame.times(0.5).plus(centerOfFormation);
    //         Translation2d newMasterTranslation = displacementGlobalFrame.times(-0.5).plus(centerOfFormation);

    //         Pose2d newMasterPose = new Pose2d(newMasterTranslation, newMasterRotation);
    //         Pose2d newSlavePose = new Pose2d(newSlaveTranslation, newSlaveRotation);
    //         updateCurrentRobot(new TimestampedVisionUpdate(newMasterPose,timestamp,stdDev));
    //         updateOtherRobot(new TimestampedVisionUpdate(newSlavePose,timestamp,stdDev));


    //         // switch (caller) {
    //         //     case master:
    //         //         masterVisionUpdatePublisher.set(newMasterPose);
    //         //         break;
    //         //     case slave:
    //         //         slaveVisionUpdatePublisher.set(newSlavePose);
    //         //         break;
    //         // }

    //         globalDisplacementPublisher.set(new Pose2d(displacementGlobalFrame.times(-0.5).plus(centerOfFormation), new Rotation2d()));
    //     } else {
    //         // send vision offset data to master to process
    //         offsetPublisher.set(update);
    //     }

    // }

    /** 
     * updates both the master and slave equally in opposite directions
     * updates masterWing and slaveWing pose to follow
     * requires master pose, slave pose, slave/master new pose
     * 
     * Notes:
     * calculated exclusively on master and sent to slave
     * used for offsets between robots
     * @param update measured displacement between robots along with timestamp of measurement where the difference is CURRENT-ROBOT-RELATIVE
     */
    private synchronized void updateLocalVision(TimestampedVisionUpdate update, RobotType caller) {
        if (!Constants.IS_MASTER) {
            timeOfLastLocalUpdate = Timer.getFPGATimestamp();

            double timestamp = update.timestamp;
            double ambiguity = update.stdDev;
            Rotation2d rotation = update.rotation.unaryMinus();
            Translation2d translation = update.translation.unaryMinus().rotateBy(rotation);
            

            Pose2d masterPosition = getOtherRobotPosition(timestamp);
            // Pose2d slavePosition = getSlavePosition(timestamp);

            // Rotation2d averageHeading = slavePosition.getRotation().plus(slavePosition.getRotation()).times(0.5);

            // Rotation2d newMasterRotation = averageHeading.plus(rotation.times(0.5));
            // Rotation2d newSlaveRotation = averageHeading.plus(rotation.times(-0.5));

            // Translation2d centerOfFormation = slavePosition.getTranslation().plus(slavePosition.getTranslation()).times(0.5);

            // Translation2d displacementGlobalFrame = translation.rotateBy(newMasterRotation);

            // Translation2d newSlaveTranslation = displacementGlobalFrame.times(0.5).plus(centerOfFormation);
            // Translation2d newMasterTranslation = displacementGlobalFrame.times(-0.5).plus(centerOfFormation);

            Pose2d newPose = new Pose2d(masterPosition.getTranslation().plus(translation.rotateBy(masterPosition.getRotation())), masterPosition.getRotation().plus(rotation));
            // slave-relative transform to center of master tags

            // Pose2d newPose = robotToTag.plus(VisionConstants.tagPose);
            // Pose2d newSlavePose = new Pose2d(newSlaveTranslation, newSlaveRotation);
            updateCurrentRobot(new TimestampedVisionUpdate(newPose,timestamp,ambiguity));
            // updateOtherRobot(new TimestampedVisionUpdate(newSlavePose,timestamp,ambiguity));

            // globalDisplacementPublisher.set(new Pose2d(displacementGlobalFrame.times(-0.5).plus(centerOfFormation), new Rotation2d()));

            switch (caller) {
                case master:
                    masterVisionUpdatePublisher.set(newPose);
                    break;
                case slave:
                    slaveVisionUpdatePublisher.set(newPose);
                    break;
            }
            
        } else {
            // send vision offset data to master to process
            offsetPublisher.set(update); // This is never actually used
        }
    }

    private Transform2d getTagPose(int tagID) {
        return VisionConstants.tagPoses.get(tagID);
    }

    private synchronized void updateGlobalVision(TimestampedVisionUpdate update, int tagID) {
        TimestampedVisionUpdate globalUpdate = new TimestampedVisionUpdate(getTagPose(tagID).plus(new Transform2d(update.translation, update.rotation)), update.timestamp, update.stdDev);
        // System.out.println(tagID + " " +  getTagPose(tagID).getX() +  " " +  getTagPose(tagID).getX() +  " " +  getTagPose(tagID).getRotation().getDegrees());
        updateGlobalVision(globalUpdate);
    }

    /** 
     * updates master, slave, and wing poses equally
     * requires current robot absolute vision measurement and current robot pose
     * 
     * Notes:
     * calculates residual as intermediate step and sends residual to all estimators for update
     * used for global wall tags
     * @param update measured absolute position of current robot along with timestamp of measurement
     */
    private synchronized void updateGlobalVision(TimestampedVisionUpdate update) {
        // Pose2d currentRobotPose = getCurrentRobotPosition(update.timestamp);
        // Pose2d updateRobotPose = new Pose2d(update.translation, update.rotation);

        // Pose2d otherRobotPose = getOtherRobotPosition(update.timestamp);

        // Transform2d robotOffset = otherRobotPose.minus(currentRobotPose);

        // Pose2d otherRobotNewPose = updateRobotPose.plus(robotOffset).interpolate(otherRobotPose, 0.5);
        
        boolean inTandem = Math.abs(timeOfLastLocalUpdate - Timer.getFPGATimestamp()) < 1;

        if (inTandem) {
            if(Constants.IS_MASTER) {
                updateCurrentRobot(update);
            } else {
                // updateOtherRobot(new TimestampedVisionUpdate(otherRobotNewPose, update.timestamp, update.stdDev));
                // System.out.println("[PhotonVision]: Skipping global vision update on slave");
            }
        } else {
            updateCurrentRobot(update);
        }
        // updateCurrentRobot(update); // TODO: this pose2d and transform2d math is definitely wrong // TODO Confirmed wrong via testing // TODO change so that it maintains the formation estimate
        // updateOtherRobot(new TimestampedVisionUpdate(otherRobotNewPose, update.timestamp, update.stdDev)); // TODO: E is a random constant

    }
    // /** switches origin and target */
    // private Transform2d switchFrameOfReference(Transform2d to) {
    //     Translation2d targetRelativeTranslation = to.getTranslation();
    //     Rotation2d targetRelativeRotation = to.getRotation(); // this should be equal to masterHeading - slaveHeading or something similar

    //     Rotation2d cameraRelativeRotation = targetRelativeRotation.unaryMinus();

    //     Translation2d cameraRelativeRotation = ;
    //     Transform2d sd;
    //     sd.inverse();
        

    //     Rotation2d slaveHeading = getOtherRobotPosition(to.timestamp).getRotation();
    //     Rotation2d masterHeading = getCurrentRobotPosition(to.timestamp).getRotation();

    //     Translation2d masterRelativeTranslation = slaveRelativeTranslation.rotateBy(slaveHeading.minus(masterHeading)); // TODO: CHECK
        
    //     Pose2d masterRelativePose = new Pose2d(masterRelativeTranslation, slaveRelativeRotation.unaryMinus());
    //     return masterRelativePose;
    // }



    /**
     * CameraThread handles vision processing in a separate thread.
     * Grabs camera results, computes pose, and updates vision measurements.
     */
    private class CameraThread extends Thread {
        // private EstimatedRobotPose pose = new EstimatedRobotPose(new Pose3d(), 0, List.of(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        // private PhotonPoseEstimator poseEstimator;
        private PhotonCamera camera;
        private final Transform3d cameraPosition;
        private Double averageDistance = 0d;
        private CameraName camName;
        // private Tuple<Transform2d, Double> updates;
        private boolean hasTarget = false;

        private  BooleanPublisher hasTargetPublisher;
        private  DoublePublisher targetsFoundPublisher;
        private  DoublePublisher timestampPublisher;
        // private final DoublePublisher distancePublisher;
        private  StructPublisher<Pose2d> camPosePublisher;

        public boolean cameraInitialized = false;

        CameraThread(CameraName camName, Transform3d cameraPosition) {
            this.camName = camName;
            this.cameraPosition = cameraPosition;

            initializeCamera();

            // Initialize camera-specific telemetry
            hasTargetPublisher = NetworkTableInstance.getDefault().getTable("Cameras").getSubTable(camName.toString()).getBooleanTopic("hasTarget").publish();
            targetsFoundPublisher = NetworkTableInstance.getDefault().getTable("Cameras").getSubTable(camName.toString()).getDoubleTopic("targetsFound").publish();
            timestampPublisher = NetworkTableInstance.getDefault().getTable("Cameras").getSubTable(camName.toString()).getDoubleTopic("timestamp").publish();
            camPosePublisher = NetworkTableInstance.getDefault().getTable("Cameras").getSubTable(camName.toString()).getStructTopic("cam pose", Pose2d.struct).publish();
        }

        Transform2d convertToRobotToRobot(Transform2d robotToTag) {
            return robotToTag.plus(VisionConstants.tagPose.inverse());
        }

        @Override
        public void run() {
            try {
                //wait for the camera to bootup before initialization
                sleep(3000);
            } catch (InterruptedException e) {
                DataLogManager.log(camName.toString() + " sleep inital failed");
            }

            //main loop for the camera thread
            while (true) {
                if (!cameraInitialized) {
                    initializeCamera();
                } else {
                    wingPoseEstimator.periodic();

                    Transform2d robotToTag = new Transform2d();
                    double timestamp = 0;

                    // main call to grab a set of results from the camera
                    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

                    double numberOfResults = results.size(); //double to prevent integer division errors
                    double totalDistances = 0;
                    boolean hasTarget = false;
                    double ambiguity = 0; //unused for now

                    //fundamentally, this loop updates the pose and distance for each result. It also logs the data to shuffleboard
                    //this is done in a thread-safe manner, as global variables are only updated at the end of the loop (no race conditions)
                    for (PhotonPipelineResult result : results) {
                        // drivetrain.updateOdo();

                        timestamp = result.getTimestampSeconds();


                        // first, do multitag as that should be done only once per result
                        // if the AprilTags.json is set up correctly, this should automatically isolate to RobotTags
                        var multiTagUpdate = doMultiTagUpdate(result);
                        if (multiTagUpdate.isPresent()) {
                            var tuple = multiTagUpdate.get();
                            robotToTag = tuple.k;
                            ambiguity = tuple.v;

                            Transform2d robotToRobot = convertToRobotToRobot(robotToTag);
                            updateLocalVision(new TimestampedVisionUpdate(robotToRobot, timestamp, ambiguity), RobotType.slave); // TODO: trigger based on offsetSubscriber
                        }

                        List<Tuple<TimestampedVisionUpdate, Integer>> globalTagList = new ArrayList<>();


                        for(PhotonTrackedTarget target : result.getTargets()) {
                            // the local hasTarget variable will turn true if ANY PipelineResult within this loop has a target
                            hasTarget = true;

                            //primary tag isolation switch
                            if(VisionConstants.RobotTagIDs.contains(target.getFiducialId())) {
                                // if there are no multitag results, fall back to single tag
                                if(multiTagUpdate.isEmpty()) {
                                    //fall back to single tag
                                    var singleTagUpdate = doSingleTagUpdate(target);
                                    if (singleTagUpdate.isPresent()) {
                                        var tuple = singleTagUpdate.get();
                                        robotToTag = tuple.k.plus(VisionConstants.tagPoses.get(target.getFiducialId())); //add tag pose offset
                                        ambiguity = tuple.v;
                                        // Transform2d robotToRobot = convertToRobotToRobotSingletag(robotToTag);
                                        // updateLocalVision(new TimestampedVisionUpdate(robotToRobot, timestamp, ambiguity)); // TODO: trigger based on offsetSubscriber
                            
                                    } else {
                                        //both methods failed
                                        DataLogManager.log("[PhotonVision] WARNING: " + camName.toString() + " both multi-tag and single-tag pose updates failed");
                                    }
                                }

                            } else if(VisionConstants.GlobalTagIDs.contains(target.getFiducialId())) {
                                // only single-tag for station tags
                                var singleTagUpdate = doSingleTagUpdate(target);
                                if (singleTagUpdate.isPresent()) {
                                    var tuple = singleTagUpdate.get();
                                    robotToTag = tuple.k;
                                    ambiguity = tuple.v;
                                    // globalTagList.add(new Tuple(new TimestampedVisionUpdate(robotToTag, timestamp, ambiguity), target.getFiducialId()));
                                    updateGlobalVision(new TimestampedVisionUpdate(robotToTag, timestamp, ambiguity), target.getFiducialId()); // TODO: specify which tag or something cuz we have insufficient information right now                               
                                } else {
                                    //single-tag method failed
                                    DataLogManager.log("[PhotonVision] WARNING: " + camName.toString() + " single-tag pose update failed for station tag");
                                }

                            } else if(VisionConstants.WingTagIDs.contains(target.getFiducialId())) {
                                // only single-tag for wing tags
                                var singleTagUpdate = doSingleWingTagUpdate(target);
                                if (singleTagUpdate.isPresent()) {
                                    var tuple = singleTagUpdate.get();
                                    robotToTag = tuple.k;
                                    ambiguity = tuple.v;


                                    updateWingVision(new TimestampedVisionUpdate(robotToTag, timestamp, ambiguity), target.getFiducialId());
                                } else {
                                    //single-tag method failed
                                    DataLogManager.log("[PhotonVision] WARNING: " + camName.toString() + " single-tag pose update failed for wing tag");
                                }

                            } else {
                                System.out.println("[PhotonVision] INFO: " + camName.toString() + " ignoring tag ID " + target.getFiducialId());
                            }

                            
                            

                            // grabs the distance to the best target (for the latest set of result)
                            totalDistances += result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();


                            camPosePublisher.set(Pose2d.kZero.plus(robotToTag)); //TODO: make more pose publishers
                        }


                        // if(globalTagList.size() > 0) {
                        //     //pick the global tag with the lowest distance
                        //     Tuple<TimestampedVisionUpdate, Integer> bestGlobalTag = globalTagList.stream().min((a, b) -> Double.compare(a.k.stdDev, b.k.stdDev)).get();
                        //     updateGlobalVision(bestGlobalTag.k, bestGlobalTag.v);
                        // }

                        //telemetry
                        hasTargetPublisher.set(result.hasTargets());
                        timestampPublisher.set(result.getTimestampSeconds());
                        targetsFoundPublisher.set(numberOfResults);
                        posePublisher.set(drivetrain.getPose());
                    }

                    if(!Constants.IS_MASTER) {
                        //convert from NT to our local timestamped format
                        List<TimestampedObject<TimestampedVisionUpdate>> NTTimestampedUpdates = List.of(offsetSubscriber.readQueue());
                        List<TimestampedVisionUpdate> timestampedOffsets = NTTimestampedUpdates.stream().map(update -> update.value).toList();
                        for (TimestampedVisionUpdate to : timestampedOffsets) {
                            Translation2d masterRelativeTranslation = to.translation;
                            Rotation2d masterRelativeRotation = to.rotation; // this should be equal to masterHeading - slaveHeading or something similar

                            // Rotation2d slaveHeading = getOtherRobotPosition(to.timestamp).getRotation();
                            // Rotation2d masterHeading = getCurrentRobotPosition(to.timestamp).getRotation();

                            // Translation2d masterRelativeTranslation = slaveRelativeTranslation.rotateBy(slaveHeading.minus(masterHeading)); // TODO: CHECK
                            
                            Transform2d slaveRelativePose = new Transform2d(masterRelativeTranslation, masterRelativeRotation).inverse();
                            // new Pose2d(masterRelativeTranslation.unaryMinus().rotateBy(masterRelativeRotation.unaryMinus()), masterRelativeRotation.unaryMinus());
                            updateLocalVision(new TimestampedVisionUpdate(slaveRelativePose, to.timestamp, to.stdDev), RobotType.master);
                            // commented for the moment to debugx later steps
                        }
                    }

                    //convert from NT to our local timestamped format
                    List<TimestampedObject<TimestampedVisionUpdate>> NTTimestampedUpdates = List.of(updateRequestSubscriber.readQueue());
                    List<TimestampedVisionUpdate> timestampedOffsets = NTTimestampedUpdates.stream().map(update -> update.value).toList();
                    for (TimestampedVisionUpdate tur : timestampedOffsets) {
                        updateCurrentRobot(tur);
                    }
                }
                try {
                    sleep(5);
                } catch (InterruptedException e) {
                    DataLogManager.log(camName.toString() + " sleep failed"); //this will never happen
                }
            }
        }

        /**
         * Returns the most recent pose and average distance to the best target <p>
         *
         * using a tuple as a type-safe alternative to the classic "return an array" (i hate java) <p>
         * this is also thread-safe, and will onlu return the most recent values from the same timestamp <p>
         *
         * @return Tuple<EstimatedRobotPose, Double> - the most recent pose and average distance to the best target
         */
        // public Tuple<Transform2d, Double> getUpdates() {
        //     return updates;
        // }

        /**
         * Returns if the camera (during latest loop cycle) has a target
         *
         * This is separate from the value on the NetworkTables, as this value is updated for the entire loop cycle <p>
         * there is an edge case where a target is found, but the pose is not updated <p>
         * likewise, there is an edge case where a target is found and then lost, but the pose is updated <p>
         * to solve this, the hasTarget value is marked as true iff any update has been sent to the estimator <p>
         * @ImplNote this is NOT strictly synchronized with the value from {@link #getUpdates()}; be careful when using this value. should use PhotonVision's hasTarget() function for most cases
         * @return has a target or not
         */
        public boolean hasTarget() {
            return hasTarget;
        }

        /**
         * Performs a multi-tag pose update using the given pipeline result.
         * @param result The pipeline result containing detected targets. ASSUMES NOT EMPTY!!
         * @return An Optional containing the computed Transform3d and ambiguity if successful; empty otherwise.
         */
        private Optional<Tuple<Transform2d, Double>> doMultiTagUpdate(PhotonPipelineResult result) {
            if (result.multitagResult.isPresent()) {
                var multiTag = result.multitagResult.get();
                // Use getCameraToRobot() to get the transform from camera to robot
                Transform3d cameraToRobot3d = multiTag.estimatedPose.best.plus(cameraPosition.inverse()).inverse();
                Transform2d cameraToRobot = new Transform2d(cameraToRobot3d.getTranslation().toTranslation2d(), cameraToRobot3d.getRotation().toRotation2d());
                double stdDev = getStdDev(cameraToRobot3d, multiTag.estimatedPose.ambiguity, -1) + 0.03;

                if(cameraToRobot.getTranslation() == null || cameraToRobot.getRotation() == null || (cameraToRobot.getRotation().getSin() == 0 && cameraToRobot.getRotation().getCos() == 0)) {
                    DataLogManager.log("[PhotonVision] ERROR: " + camName.toString() + " single-tag pose update returned null values");
                    return Optional.empty();
                }

                
                return Optional.of(new Tuple<Transform2d, Double>(cameraToRobot, stdDev));
            }
            //else
            // DataLogManager.log("[PhotonVision] INFO: " + camName.toString() + " multitag result requested but not present");
            return Optional.empty();
        }

        private Optional<Tuple<Transform2d, Double>> doSingleTagUpdate(PhotonTrackedTarget target) {
            //grabs the target pose, relative to the camera, and compensates for the camera position
            Transform3d cameraToRobot3d = cameraPosition.plus(target.getBestCameraToTarget());
            Transform2d cameraToRobot = new Transform2d(cameraToRobot3d.getTranslation().toTranslation2d(), cameraToRobot3d.getRotation().toRotation2d()).inverse();
            double stdDev = getStdDev(cameraToRobot3d, target.getPoseAmbiguity(), target.getFiducialId()) + 0.5; // slightly lower stddev for single tags

            if(cameraToRobot.getTranslation() == null || cameraToRobot.getRotation() == null || (cameraToRobot.getRotation().getSin() == 0 && cameraToRobot.getRotation().getCos() == 0)) {
                DataLogManager.log("[PhotonVision] ERROR: " + camName.toString() + " single-tag pose update returned null values");
                return Optional.empty();
            }


            return Optional.of(new Tuple<Transform2d, Double>(cameraToRobot, stdDev));
        }
        /** 
         * hardcoded solution to solve geometry for tags on the ceiling
         */
        private Optional<Tuple<Transform2d, Double>> doSingleWingTagUpdate(PhotonTrackedTarget target) {
            //grabs the target pose, relative to the camera, and compensates for the camera position
            Transform3d cameraToRobot3d = cameraPosition.plus(target.getBestCameraToTarget());
            Transform2d cameraToRobot = new Transform2d(cameraToRobot3d.getTranslation().toTranslation2d(), new Rotation2d(target.getBestCameraToTarget().getRotation().getMeasureX().negate())).inverse();
            double stdDev = getStdDev(cameraToRobot3d, target.getPoseAmbiguity(), target.getFiducialId()) + 0.5;

            if(cameraToRobot.getTranslation() == null || cameraToRobot.getRotation() == null || cameraToRobot.getRotation().getSin() == 0 || cameraToRobot.getRotation().getCos() == 0) {
                DataLogManager.log("[PhotonVision] ERROR: " + camName.toString() + " single-tag pose update returned null values");
                return Optional.empty();
            }


            return Optional.of(new Tuple<Transform2d, Double>(cameraToRobot, stdDev));
        }

        private double getStdDev(Transform3d distance, double ambiguity, int tagID) {
            // if(tagID != -1) { //if not multitag
            //     if(Math.abs(distance.inverse().getTranslation().toTranslation2d().getAngle().getDegrees()) < 5) {
            //         //flat tag
            //         return 10;
            //     }
            // }

            final double distanceK = 0.03;
            if(ambiguity <= 0.05) {
                return distanceK * distance.getTranslation().getNorm();
            } else if (ambiguity < 0.2) {
                return distanceK * distance.getTranslation().getNorm() * 3; //3x the stddev at or before half ambiguity
            } else {
                //bad vision update, return 10m stddev
                // DataLogManager.log("[PhotonVision] WARNING: " + camName.toString() + " pose ambiguity is high for tag " + tagID + ": " + ambiguity);
                return 10;
            }
        }

        private PhotonCamera getCameraObject() {
            return camera;
        }

        private void initializeCamera() {
                camera = new PhotonCamera(camName.toString());
                cameraInitialized = true;
        }

        // Initializes the camera with the given name. This is intended for the master robot, if it is not running pose estimation.
        // The master robot runs the TimeServer for the slave(s), so it is still required for the camera to be initialized.
        @SuppressWarnings("resource")
        public static void initializeCamera(String cameraName) {
            new PhotonCamera(cameraName.toString());
        }
    }
}

    /* original single-camera vision update methods
    private synchronized void updateLocalVision() {
        Tuple<EstimatedRobotPose, Double> update = camThread.getUpdates();
        timestampedMasterPoses.addAll(List.of(masterPoseSubscriber.readQueue()));

        //trim timestampedMasterPoses to 10 values (for efficiency)
        if (timestampedMasterPoses.size() > 10) {
            timestampedMasterPoses = timestampedMasterPoses.subList(timestampedMasterPoses.size() - 10, timestampedMasterPoses.size());
        }

        //find the master pose with the closest timestamp to the camera's timestamp
        //NT timestamps are measured in microseconds, PhotonVision timestamps are seconds. Mulitiply by one million to convert/
        double visionTimeStampMicroSeconds = update.v * 1000000d;
        closestMasterPose = timestampedMasterPoses.stream()
            .min((a, b) -> Double.compare(Math.abs(a.serverTime - visionTimeStampMicroSeconds), Math.abs(b.serverTime - visionTimeStampMicroSeconds)))
            .map(pose -> pose.value)
            .orElse(null);
        
        
        //account for tag-to-robot and camera-to-robot offsets and then combine vision measurement with master odometry position
        //hey so actually have zero clue how this works!!
        Translation2d x = update.k.getTranslation().plus(RobotConstants.centerOfMasterToTag).rotateBy(drivetrain.getPose().getRotation()).rotateBy(Rotation2d.k180deg);
        Rotation2d t = update.k.getRotation().unaryMinus().rotateBy(Rotation2d.kCW_90deg);
        Transform2d visionTranslation = new Transform2d(x, t);

        Pose2d fieldRelativePose = new Pose2d(closestMasterPose.getTranslation().plus(visionTranslation.getTranslation()), closestMasterPose.getRotation().plus(visionTranslation.getRotation()));


        // add the master pose to the translation to get field-relative pose. 
        // grab the timestamp
        // grab the distance to the best tag
        drivetrain.addVisionMeasurement(fieldRelativePose, update.v, update.k.getTranslation().getNorm());

        posePublisher.accept(fieldRelativePose);
        pose = fieldRelativePose;

    }
        */
