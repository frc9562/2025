package frc.robot.commands.SwerveCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

import java.util.List;
import java.util.Optional;

public class PathPlannerAlignment extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final boolean isLeftBranchUsed;

    private final AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private PathPlannerPath path;
    private Command followCommand;

    public PathPlannerAlignment(CommandSwerveDrivetrain drivetrain,
                                VisionSubsystem vision,
                                boolean isLeftBranchUsed) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.isLeftBranchUsed = isLeftBranchUsed;

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        // Must have a target and tag pose from vision
        if (vision.targeta == null || vision.tagPose == null) {
            cancel();
            return;
        }

        // Get tag pose from the 2025 Reefscape field layout
        Optional<Pose3d> tagPoseOpt =
                aprilTagFieldLayout.getTagPose(vision.targeta.getFiducialId());
        if (tagPoseOpt.isEmpty()) {
            cancel();
            return;
        }

        // Current robot pose (from vision/odometry fusion)
        Pose2d currentPose = vision.myPose3d != null
                ? vision.myPose3d.toPose2d()
                : new Pose2d(); // fallback, but ideally myPose3d is never null

        // Desired scoring pose based on which branch we’re using
        Pose2d targetPose = vision.calculateScoringPose(vision.tagPose, isLeftBranchUsed);

        // Build a simple 2-waypoint path: currentPose → targetPose
        path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(currentPose, targetPose),
                // Path constraints: max vel, max accel, max angular vel/accel (rad/s, rad/s^2)
                new PathConstraints(2.0, 2.0,
                        Math.toRadians(540.0), Math.toRadians(720.0)),
                null, // no custom ideal starting state
                new GoalEndState(0.0, targetPose.getRotation())
        );

        // Don’t auto-flip on red/blue; we’re already computing field-relative poses
        path.preventFlipping = true;

        // Build and schedule the followPath command
        followCommand = AutoBuilder.followPath(path);
        followCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return followCommand != null && followCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (followCommand != null) {
            followCommand.cancel();
        }
    }
}
