// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
// import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private PhotonCamera camera1;
  // private PhotonCamera camera2;
  // private PhotonCamera camera3;
  // private PhotonCamera camera4;
  // private PhotonCamera[] cameras;

  private PhotonPipelineResult result;

  public final Transform3d cameraPositions;

  private static final Map<PhotonCamera, Double> CAMERA_HEIGHTS = new HashMap<>();
  private static final Map<PhotonCamera, Double> CAMERA_PITCHES = new HashMap<>();
  private static Set<Integer> ID_SET;

  private PIDController xController = new PIDController(1.5,0,0); //forward
  private PIDController yController = new PIDController(1.5,0,0); // strafe
  private PIDController thetaController = new PIDController(3.0,0,0); // turn

  {
      // Angular PID must wrap around [-180, 180]
      thetaController.enableContinuousInput(-180, 180);
  
      // Set tolerances for autoalign
      xController.setTolerance(0.05);     // 5cm
      yController.setTolerance(0.05);     // 5cm
      thetaController.setTolerance(1.0);  // 1 degree
  
      // OPTIONAL (recommended)
      xController.setIntegratorRange(-0.3, 0.3);
      yController.setIntegratorRange(-0.3, 0.3);
      thetaController.setIntegratorRange(-0.4, 0.4);
  }

  PhotonTrackedTarget closestTarget = null;
  PhotonCamera bestCamera = null;
  double targetHeight;
  double distance;
  double newDist = Double.MAX_VALUE;

  public Pose3d robotPose;
  public Pose3d myPose3d = null;

  public Pose2d tagPose = null;
  public ChassisSpeeds currentVelocity = null;
  public PhotonTrackedTarget targeta = null;

  long iteration = 0;
  String oldIdsString = "";

  PhotonPoseEstimator camera1PoseEstimator;
  // PhotonPoseEstimator camera2PoseEstimator;
  // PhotonPoseEstimator camera3PoseEstimator;
  // PhotonPoseEstimator camera4PoseEstimator;

  private static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2025ReefscapeWelded);
  public final static CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;

  public VisionSubsystem() {
    this.cameraPositions = VisionConstants.camera1ToRobot;
    // this.cameraPositions[1] = VisionConstants.camera2ToRobot;
    // this.cameraPositions[2] = VisionConstants.camera3ToRobot;
    // this.cameraPositions[3] = VisionConstants.camera4ToRobot;

    this.camera1 = new PhotonCamera(VisionConstants.cameraName1);
    // this.camera2 = new PhotonCamera(VisionConstants.cameraName2);
    // this.camera3 = new PhotonCamera(VisionConstants.cameraName3);
    // this.camera4 = new PhotonCamera(VisionConstants.cameraName4);
    // this.cameras = new PhotonCamera[] {camera1};

    // this.result = new PhotonPipelineResult (camera1.getLatestResult());
    this.result = new PhotonPipelineResult();

    CAMERA_HEIGHTS.put(camera1, VisionConstants.camera1Z);
    // CAMERA_HEIGHTS.put(camera2, VisionConstants.camera2Z);
    // CAMERA_HEIGHTS.put(camera3, VisionConstants.camera3Z);
    // CAMERA_HEIGHTS.put(camera4, VisionConstants.camera4Z);

    CAMERA_PITCHES.put(camera1, VisionConstants.camera1pitch);
    // CAMERA_PITCHES.put(camera2, VisionConstants.camera2pitch);
    // CAMERA_PITCHES.put(camera3, VisionConstants.camera3pitch);
    // CAMERA_PITCHES.put(camera4, VisionConstants.camera4pitch);

    camera1PoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        VisionConstants.camera1ToRobot);
    // camera2PoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
    // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.camera2ToRobot);
    // camera3PoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
    // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.camera3ToRobot);
    // camera4PoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
    // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.camera4ToRobot);

  }

  public void findBestCameraToTarget() { // find closest apriltag
    newDist = Double.MAX_VALUE;
    result = camera1.getLatestResult();

    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();

      double CamHeight = CAMERA_HEIGHTS.get(camera1); // fix this one
      double CamPitch = CAMERA_PITCHES.get(camera1); // fix this one

      for (PhotonTrackedTarget target : targets) {
        // distance also used for gotobesttarget
        distance = PhotonUtils.calculateDistanceToTargetMeters(CamHeight,
            aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getZ(), CamPitch, target.getPitch());

        if (distance < newDist) {
          newDist = distance;
          closestTarget = target;
        }
      }
    }

  }

  public Pose2d getOldPose() {
    if (robotPose != null) {
      return robotPose.toPose2d();
    } else {
      return null;
    }
  }

  public PhotonTrackedTarget getClosestTarget() { // from all cameras (not just one)
    findBestCameraToTarget();
    return closestTarget;
  }

  public double getBestYaw() { // get yaw of closest apriltag
    return closestTarget.getYaw();
  }

  public double getTargetDistance(int targetid) { // any error returns -1
    result = camera1.getLatestResult();
    PhotonTrackedTarget targetBababoey;

    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      for (PhotonTrackedTarget target : targets) {
        if (target.getFiducialId() == targetid) {
          targetBababoey = target;
          double CamHeight = CAMERA_HEIGHTS.get(camera1);
          double CamPitch = CAMERA_PITCHES.get(camera1);
          return PhotonUtils.calculateDistanceToTargetMeters(CamHeight,
              aprilTagFieldLayout.getTagPose(targetid).get().getZ(), CamPitch, targetBababoey.getPitch());
        }
      }
      return -1;
    } else
      return -1;
  }

  public void findBestCameraToPOI(String tagType) { // find closest apriltag of type
    newDist = Double.MAX_VALUE;

    if (tagType.toLowerCase().equals("coral")) {
      targetHeight = VisionConstants.CORAL_STATION_TAG;
      ID_SET = VisionConstants.Coral_ID;
    }

    if (tagType.toLowerCase().equals("processor")) {
      targetHeight = VisionConstants.PROCESSOR_TAG;
      ID_SET = VisionConstants.Processor_ID;
    }

    if (tagType.toLowerCase().equals("reef")) {
      targetHeight = VisionConstants.REEF_TAG;
      ID_SET = VisionConstants.Reef_ID;
    }

    if (tagType.toLowerCase().equals("barge")) {
      targetHeight = VisionConstants.BARGE_TAG;
      ID_SET = VisionConstants.Barge_ID;
    }

    result = camera1.getLatestResult();

    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();

      double CamHeight = CAMERA_HEIGHTS.get(camera1);
      double CamPitch = CAMERA_PITCHES.get(camera1);

      for (PhotonTrackedTarget target : targets) {

        if (ID_SET.contains(target.getFiducialId())) {

          distance = PhotonUtils.calculateDistanceToTargetMeters(CamHeight, targetHeight, CamPitch,
              target.getPitch());

          if (distance < newDist) {
            newDist = distance;
            closestTarget = target;
          }
        }
      }
    }

  }

  public PhotonTrackedTarget getBestTarget() { // find closest apriltag to specific camera
    // Check if the cameraNum is valid

    result = camera1.getLatestResult();

    if (result == null || !result.hasTargets()) {
      // System.out.println("[WARN] Camera " + 1 + " returned NULL result.");
      return null;
    }

    PhotonTrackedTarget bestTarget = result.getBestTarget();

    if (bestTarget == null) {
      // System.out.println("[ERROR] Camera " + 1 + " hasTargets() was TRUE but getBestTarget() is NULL.");
      return null;
    }

    // System.out.println("[INFO VISION-SUB] Camera " + 1 + " detected AprilTag with Yaw: " + bestTarget.getYaw() + ", and id: " + bestTarget.getFiducialId());
    return bestTarget;
  }

  /*
   * public Optional<Integer[]> getIdsDetectedByCamera(int cameraNum) {
   * if (cameraNum < 0 || cameraNum >= cameras.length) {
   * // System.out.println("[ERROR] Invalid camera index: " + cameraNum);
   * return Optional.empty();
   * }
   * 
   * PhotonPipelineResult results = cameras[cameraNum].getLatestResult();
   * if (results == null) {return Optional.empty();}
   * List<PhotonTrackedTarget> targets = results.getTargets();
   * if (targets == null) {return Optional.empty();}
   * 
   * ArrayList<Integer> ids = new ArrayList<Integer>();
   * 
   * for (PhotonTrackedTarget target : targets) {
   * if (target.getFiducialId() != -1) {
   * ids.add(target.getFiducialId());
   * }
   * }
   * 
   * Integer[] output = new Integer[ids.size()];
   * output = ids.toArray(output);
   * 
   * return Optional.of(output);
   * }
   * 
   * public Optional<Integer[]> getIdsDetectedByAllCameras() {
   * ArrayList<Integer> ids = new ArrayList<Integer>();
   * for (int cameraId=0; cameraId<cameras.length; cameraId++) {
   * Optional<Integer[]> idsOfCamera = getIdsDetectedByCamera(cameraId);
   * if (idsOfCamera.isPresent()) {
   * for (Integer id : idsOfCamera.get()) {
   * if (!ids.contains(id)) {
   * ids.add(id);
   * }
   * }
   * }
   * }
   * Integer[] output = new Integer[ids.size()];
   * output = ids.toArray(output);
   * return Optional.of(output);
   * }
   * 
   * public HashMap<Integer, Optional<Integer[]>>
   * getIdsDetectedByAllCamerasByCamera() {
   * HashMap<Integer, Optional<Integer[]>> map = new HashMap<Integer,
   * Optional<Integer[]>>();
   * for (int cameraId=0; cameraId<cameras.length; cameraId++) {
   * map.put(cameraId, getIdsDetectedByCamera(cameraId));
   * }
   * return map;
   * }
   */
  public Pose2d estimatePose(Pose2d oldPose) { // estimate robot pose based on camera
    PhotonTrackedTarget bababoey = getBestTarget();
    if (bababoey != null) {

      if (aprilTagFieldLayout.getTagPose(bababoey.getFiducialId()).isPresent()) {
        robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
            bababoey.getBestCameraToTarget(),
            aprilTagFieldLayout.getTagPose(bababoey.getFiducialId()).get(),
            cameraPositions);
        return robotPose.toPose2d();
      } else
        return oldPose;
    }

    else
      return oldPose;

  }

  public Pose2d calculateScoringPose(Pose2d tagPose, boolean isLeftBranch) {
    // Distance from tag to the center of the reef's scoring branch
    double REEF_OFFSET = 0.1651; // in meters (measured on CAD)
    double ARM_OFFSET = 0.2511425; // same

    double TOTAL_OFFSET = REEF_OFFSET - ARM_OFFSET; // Adjust for the arm being off-center

    // (+) Y = left in WPILib coordinates
    double yOffset = isLeftBranch ? TOTAL_OFFSET : -TOTAL_OFFSET;

    Translation2d offset = new Translation2d(0, yOffset).rotateBy(tagPose.getRotation());

    return new Pose2d(
        tagPose.getTranslation().plus(offset),
        tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
  }
  /*
   * public Pose2d getEstimatedRobotPose() {
   * var result = camera1PoseEstimator.update();
   * if (result.isPresent()) {
   * return result.get().estimatedPose.toPose2d();
   * }
   * return new Pose2d();
   * }
   */

  /*
   * public Command turnToBestTarget(int camNum) {
   * // double targetID = getBestTarget(camNum).getFiducialId(); // i dont think i
   * need this idk
   * return this
   * .run(() ->
   * RobotContainer.drivetrain.setControl(RobotContainer.drive.withRotationalRate(
   * -1 * MaxAngularRate)));
   * }
   */

  /*
   * public void turnToBestTarget(int camNum){ // int camNum ex: 2, target = 1
   * double targetID = getBestTarget(camNum).getFiducialId();
   * double offYaw = closestTarget.getYaw(); // distance from closest target to
   * cam center
   * //RobotContainer.drivetrain.setControl(RobotContainer.drive.
   * withRotationalRate(-1 * MaxAngularRate));
   * }
   */
  /*
   * public Command setElevatorHeight(double targetHeight) {
   * this.target = targetHeight;
   * return this.run(() -> pid.setReference(getError(targetHeight * 2),
   * ControlType.kMAXMotionPositionControl, slot0));
   * } // could this be the error
   */

  public void estimatePoseMultitag(SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
    camera1PoseEstimator.setReferencePose(swerveDrivePoseEstimator.getEstimatedPosition());
    // camera2PoseEstimator.setReferencePose(swerveDrivePoseEstimator.getEstimatedPosition());
    // camera3PoseEstimator.setReferencePose(swerveDrivePoseEstimator.getEstimatedPosition());
    // camera4PoseEstimator.setReferencePose(swerveDrivePoseEstimator.getEstimatedPosition());
    EstimatedRobotPose camera1EstimatedPose;
    // EstimatedRobotPose camera2EstimatedPose;
    // EstimatedRobotPose camera3EstimatedPose;
    // EstimatedRobotPose camera4EstimatedPose;

    try {
      camera1EstimatedPose = camera1PoseEstimator.update(camera1.getLatestResult()).get();
      // camera2EstimatedPose =
      // camera2PoseEstimator.update(camera2.getLatestResult()).get();
      // camera3EstimatedPose =
      // camera3PoseEstimator.update(camera3.getLatestResult()).get();
      // camera4EstimatedPose =
      // camera4PoseEstimator.update(camera4.getLatestResult()).get();
      swerveDrivePoseEstimator.addVisionMeasurement(camera1EstimatedPose.estimatedPose.toPose2d(),
          Timer.getFPGATimestamp());
      // swerveDrivePoseEstimator.addVisionMeasurement(camera2EstimatedPose.estimatedPose.toPose2d(),
      // Timer.getFPGATimestamp());
      // swerveDrivePoseEstimator.addVisionMeasurement(camera3EstimatedPose.estimatedPose.toPose2d(),
      // Timer.getFPGATimestamp());
      // swerveDrivePoseEstimator.addVisionMeasurement(camera4EstimatedPose.estimatedPose.toPose2d(),
      // Timer.getFPGATimestamp());
    }

    catch (Exception e) {
    }
  }

  @Override
  public void periodic() {
    iteration += 1;
    if (iteration % 50 == 0) {
      // oldIdsString =
      // Arrays.toString(getIdsDetectedByAllCameras().get()).replace("[",
      // "").replace("]", "");
    }
    SmartDashboard.putString("Vision/Detected IDs: ", oldIdsString);
    if (closestTarget != null && bestCamera != null) {
      // System.out.println("SOMETHING IS FOUND!!!!!!!!!");
      SmartDashboard.putString("Vision/Using Camera: ", bestCamera.getName());
      SmartDashboard.putNumber("Vision/Best Yaw: ", closestTarget.getYaw());
      SmartDashboard.putNumber("Vision/Best Pitch: ", closestTarget.getPitch());
      SmartDashboard.putNumber("Vision/Best Area: ", closestTarget.getArea());
      SmartDashboard.putNumber("Vision/Exact Distance (meters): ", newDist);
    } else {
      SmartDashboard.putString("Vision/Using Camera: ", "No Target Found");
    }
    try {
      targeta = getClosestTarget();

      // get the closest tag -> fix this
      if (targeta == null) {
        // System.out.println("[VisAutoAlign] No target detected.");
      } else {
        try {
          if (aprilTagFieldLayout.getTagPose(targeta.getFiducialId()).isPresent()) {
            myPose3d = PhotonUtils.estimateFieldToRobotAprilTag(targeta.getBestCameraToTarget(),
                aprilTagFieldLayout.getTagPose(targeta.getFiducialId()).get(), cameraPositions);
          }
        } catch (Exception e) {
          // System.out.println("[ERORR] tralalero tralala?: " + e);
        }
        tagPose = estimatePose(getOldPose()); // Optional override: use AprilTagFieldLayout
        // calculates the scoring pose using the tag's pose
        currentVelocity = drivetrain.getKinematics().toChassisSpeeds(
            drivetrain.getModules()[0].getCurrentState(), drivetrain.getModules()[1].getCurrentState(),
            drivetrain.getModules()[2].getCurrentState(), drivetrain.getModules()[3].getCurrentState()); // drivetrain.getKinematics().toChassisSpeeds()
        // toChassisSpeeds() has parameters: SwerveModuleState, which we dont use
        // (if anything doesnt work its probably because of this)
        // System.out.println("THIS IS THE DEGREESS: " + myPose3d.getRotation().getAngle() * 180 / Math.PI);

      }

    } catch (Exception e) {
      // System.out.println("ERROR AT VISION SUBSYSTEM PERIODIC: " + e);
    }

  }

  /**
   * Returns the AprilTag Pose relative to the robot:
   * X = forward (+ is in front of the robot)
   * Y = sideways (+ is left of the robot)
   * Z = vertical difference
   * Yaw = angle difference between robot and tag
   * Returns null if no target (to avoid any... complications)
   */

  public double[] getRobotRelativePose() {
    PhotonTrackedTarget target = getClosestTarget();
    if (target == null) return null;

    // Photon already provides this transform; yippee for a 750$ camera
    //[X, Y, Z, Roll, Pitch, Yaw] relative to robot 
    double[] pose = target.getBestCameraToTarget().getTranslation().toArray();

    double yaw = target.getYaw(); // degrees
    return new double[]{pose[0], pose[1], pose[2], yaw};
  }
 // returns full field pose if valid, otherwise null
  public Pose2d getVisionPose() {
    if (myPose3d == null) return null;
    return myPose3d.toPose2d();
  }


  public void updatePoseEstimator(SwerveDrivePoseEstimator estimator) {
    estimatePoseMultitag(estimator);


  }
  /**
 * Returns alignment error relative to the closest tag.
 * x = forward error (meters)
 * y = strafe error (meters)
 * theta = rotational error (degrees)
 * Returns null if no tag found.
 */
  public double[] getAutoAlignError() {
    PhotonTrackedTarget target = getClosestTarget();
    if (target == null) return null;

    Transform3d camToTag = target.getBestCameraToTarget();

    double x = camToTag.getX();   // forward/back
    double y = camToTag.getY();   // left/right
    double yaw = target.getYaw(); // degrees

    return new double[]{ x, y, yaw };
  }

  public PIDController getXController() {
    return xController;
  }
  
  public PIDController getYController() {
    return yController;
  }
  
  public PIDController getThetaController() {
    return thetaController;
  }
  



}
