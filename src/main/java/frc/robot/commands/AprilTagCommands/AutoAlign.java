  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot.commands.AprilTagCommands;

  import edu.wpi.first.wpilibj2.command.Command;
  import frc.robot.subsystems.VisionSubsystem;
  import frc.robot.RobotContainer;
  import frc.robot.generated.TunerConstants;
  import edu.wpi.first.math.kinematics.ChassisSpeeds;

  public class AutoAlign extends Command {

    private final VisionSubsystem vision;

    public AutoAlign(VisionSubsystem vision) {
      this.vision = vision;
      addRequirements(vision, RobotContainer.drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

      double[] errors = vision.getAutoAlignError();
      if (errors == null) {
        RobotContainer.drivetrain.setControl(
          TunerConstants.DriveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
        );
        return;
      }

      double xErr = errors[0];     // meters
      double yErr = errors[1];     // meters
      double yawErr = errors[2];   // degrees

      double xSpeed = vision.getXController().calculate(0, xErr);
      double ySpeed = vision.getYController().calculate(0, yErr);
      double turnSpeed = vision.getThetaController().calculate(0, yawErr);


      // Clamp speeds for safety
      xSpeed = Math.max(Math.min(xSpeed, 1.5), -1.5);
      ySpeed = Math.max(Math.min(ySpeed, 1.5), -1.5);
      turnSpeed = Math.max(Math.min(turnSpeed, 2.0), -2.0);

      RobotContainer.drivetrain.setControl(
          TunerConstants.DriveRequest
              .withVelocityX(xSpeed)
              .withVelocityY(ySpeed)
              .withRotationalRate(turnSpeed)
      );
    }

    @Override
    public void end(boolean interrupted) {
      RobotContainer.drivetrain.setControl(
        TunerConstants.DriveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
      );
    }

    @Override
    public boolean isFinished() {
      return vision.getXController().atSetpoint()
          && vision.getYController().atSetpoint()
          && vision.getThetaController().atSetpoint();
    }

  }

