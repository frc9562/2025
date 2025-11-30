// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AprilTagCommands.AutoAlign;
import frc.robot.commands.LEDCommands.SetLedStateCommand;
// import frc.robot.commands.SwerveCommands.PathPlannerAlignment;
// import frc.robot.commands.SwerveCommands.ManualAutoAlign;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ArmConstants.ArmAngles;
import frc.robot.constants.ArmConstants.IntakeDirection;
import frc.robot.constants.CoralGroundIntakeConstants.CoralAngles;
import frc.robot.constants.ElevatorConstants.ElevatorHeights;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LaserCanSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LedSubsystem.RobotState;
import frc.robot.subsystems.CoralGroundIntake;

public class RobotContainer {

        // speed is divided by 3 to accommodate for small testing spaces
        public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
        // top
        // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
        // second -
        // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.3)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
        // motors

        private final Telemetry logger = new Telemetry(MaxSpeed);

        static CommandXboxController XController = new CommandXboxController(0);

        public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final static ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
        public final static ArmSubsystem m_armSubsystem = new ArmSubsystem();
        public final static VisionSubsystem m_visionSubsystem = new VisionSubsystem();
        public final static LaserCanSubsystem m_laserCan = new LaserCanSubsystem();
        public final static LedSubsystem m_ledSubsystem = new LedSubsystem();
        public final static CoralGroundIntake m_coralGroundIntake = new CoralGroundIntake();

        public final static boolean toggleAlgae = false;

        // public PathPlannerAlignment pathPlannerAlignmentLeft = new
        // PathPlannerAlignment(drivetrain, m_visionSubsystem, true);
        // public PathPlannerAlignment pathPlannerAlignmentRight = new
        // PathPlannerAlignment(drivetrain, m_visionSubsystem, false);

        // private final SendableChooser<Command> autoChooser;

        // Intake Commands
        private final Command intakeCoralAlgae() {

                if (m_laserCan.processMeasurement()) {
                        return m_armSubsystem.intakeOuttake(IntakeDirection.IN)
                                        .until(() -> m_armSubsystem.getOpenCurrent() > ArmConstants.OPEN_STALL_LIMIT)
                                        .andThen(m_armSubsystem.intakeOuttake(0.4)); // find a good percent to hold the
                        // algae in
                }

                else if (!m_laserCan.processMeasurement()) {
                        return m_armSubsystem.intakeOuttake(IntakeDirection.IN)
                                        .until(() -> m_armSubsystem.getOpenCurrent() > ArmConstants.OPEN_STALL_LIMIT)
                                        .andThen(m_armSubsystem.intakeOuttake(IntakeDirection.IN).withTimeout(0.5))
                                        .finallyDo(() -> m_armSubsystem.intakeOuttake(IntakeDirection.STOP));
                }

                return m_armSubsystem.intakeOuttake(IntakeDirection.STOP);
        }

        private final Command simpleHome() {
                return m_armSubsystem.zero()
                .alongWith(m_elevatorSubsystem.rocketShip());
        }

        private final Command setHeightAngleToPOI(ArmAngles angle, ElevatorHeights height) {
                return m_armSubsystem.zero()
                                .andThen(m_elevatorSubsystem.setElevatorHeight(height.getHeight())
                                                .until(() -> m_elevatorSubsystem.isAtPoint(height.getHeight())))
                                .andThen(m_armSubsystem.setPitch(angle.getAngle())
                                                .until(() -> m_armSubsystem.isAtPoint(angle.getAngle())));
        }

        private final Command descoreAlgae(ElevatorHeights height) {
                return m_elevatorSubsystem.setDeltaHeight(height)
                                .andThen(m_armSubsystem.setPitch(ArmAngles.ALGAE))
                                .until(() -> m_armSubsystem.isAtPoint(ArmAngles.ALGAE))
                                .andThen(m_armSubsystem.intakeOuttake(IntakeDirection.OUT));
        }

        private final Command intakeFromGround() {
                return new SequentialCommandGroup(new ParallelCommandGroup(m_coralGroundIntake.intakeSequence(),
                                m_armSubsystem.setPitch(ArmAngles.CORAL)
                                                .until(() -> m_armSubsystem.isAtPoint(ArmAngles.CORAL))),
                                new ParallelRaceGroup(intakeCoralAlgae(),
                                                m_coralGroundIntake.run(() -> m_coralGroundIntake.intakeBoth())),
                                m_coralGroundIntake.run(() -> m_coralGroundIntake.stopIntake()).withTimeout(0.1),
                                m_armSubsystem.run(() -> m_armSubsystem.intakeOuttake(IntakeDirection.STOP))
                                                .withTimeout(0.1),
                                m_armSubsystem.zero());
        }
        // auto align
        public Command autoAlign() {
                return new AutoAlign(m_visionSubsystem);
            }

        // autoScore
        private final Command autoScoreL4() {
                return m_armSubsystem.zero()
                .andThen(setHeightAngleToPOI(ArmAngles.L4,
                                ElevatorHeights.L4).withTimeout(3.4))
                .andThen(m_armSubsystem.intakeOuttake(IntakeDirection.OUT).withTimeout(1.5))
                .andThen(m_armSubsystem.intakeOuttake(IntakeDirection.STOP).withTimeout(0.1))
                .andThen(simpleHome());
        }

        // private final Command manualAutoAlign = new ManualAutoAlign(drivetrain,
        // m_visionSubsystem, drive);

        /*
         * private final Command L1Score(){
         * return m_coralGroundIntake.intakeWithBeamBreak()
         * .andThen(m_coralGroundIntake.run(() ->
         * m_coralGroundIntake.stopIntake()).withTimeout(0.1))
         * .andThen(m_coralGroundIntake.setIntakePosition(CoralAngles.L1));
         * }
         */

        public RobotContainer() {

                registerCommands();

                burnAllFlash();
                configureBindings();

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        private void registerCommands() {
                // NamedCommands.registerCommand("manualAutoAlign", manualAutoAlign);
                // NamedCommands.registerCommand("pathPlannerAlignmentLeft",
                // pathPlannerAlignmentLeft);
                // NamedCommands.registerCommand("pathPlannerAlignmentRight",
                // pathPlannerAlignmentRight);

                NamedCommands.registerCommand("ScoreCoral", autoScoreL4());
                NamedCommands.registerCommand("Intake", intakeFromGround());
        }

        private void burnAllFlash() {
                m_elevatorSubsystem.burnFlash();
                m_armSubsystem.burnFlash();
                m_coralGroundIntake.burnFlash();
        }
        

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.

                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-XController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative                                                // (forward)
                                                .withVelocityY(-XController.getLeftX() * MaxSpeed) // Drive left with
                                                                                                   // negative X (left)
                                                .withRotationalRate(-XController.getRightX() * MaxAngularRate)));

                // m_armSubsystem.setDefaultCommand(m_armSubsystem.run(() ->
                // m_armSubsystem.manualPitchMotor(XController.getRightY())));
                m_ledSubsystem.setDefaultCommand(new SetLedStateCommand(m_ledSubsystem, RobotState.RAINBOW));

                m_coralGroundIntake.setDefaultCommand(m_coralGroundIntake.setIntakePosition(CoralAngles.ZERO)
                                .onlyIf(() -> m_armSubsystem.isSafe())
                                .unless(() -> m_coralGroundIntake.isAtPoint(CoralAngles.ZERO)));

                // THISONETHISONETHISONETHISONETHISONETHISONETHISONETHISONETHISONETHISONETHISONETHISONE

                // reset the field-centric heading on d-pad up
                XController.povUp().onChange(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // zero the arm on d-pad right
                XController.povRight().onChange(m_armSubsystem.zero());

                // sends the elevator up to grab a coral off the reef
                // XController.rightStick().onChange(descoreAlgae(ElevatorHeights.L3));
                // XController.leftStick().onChange(descoreAlgae(ElevatorHeights.L2));

                XController.leftBumper().onChange(setHeightAngleToPOI(ArmAngles.L3, ElevatorHeights.L3));
                XController.leftTrigger().onChange(setHeightAngleToPOI(ArmAngles.L2, ElevatorHeights.L2));
                XController.rightTrigger().onChange(setHeightAngleToPOI(ArmAngles.L4, ElevatorHeights.L4));

                XController.povDown().onChange(simpleHome());

                // intakes
                XController.rightBumper().onTrue(intakeFromGround());

                XController.a().onTrue(m_coralGroundIntake.run(() -> m_coralGroundIntake.outakeBoth()));
                XController.a().onFalse(
                                m_coralGroundIntake.runOnce(() -> m_coralGroundIntake.stopIntake()).withTimeout(0.1));

                XController.y().onTrue(m_armSubsystem.intakeOuttake(IntakeDirection.OUT)
                                .alongWith(m_ledSubsystem
                                                .run(() -> m_ledSubsystem.setState(RobotState.SHOOTING_REEF))));
                XController.y().onFalse(m_armSubsystem.intakeOuttake(IntakeDirection.STOP).withTimeout(0.1));

                XController.b().onChange(m_coralGroundIntake.L1());
                
                // AUTO ALIGN USING APRILTAG
                XController.x().whileTrue(autoAlign());


                // XController.b().onTrue(pathPlannerAlignmentLeft);
                // XController.b().onTrue(pathPlannerAlignmentRight);
                // XController.b().whileTrue(new ManualAutoAlign(drivetrain, m_visionSubsystem,
                // drive));
        }

        public Command getAutonomousCommand() {
                // return Commands.print("No autonomous command configured");
                // return new PathPlannerAuto("New Auto");
                // return new ParallelDeadlineGroup(new WaitCommand(3),
                // drivetrain.applyRequest(() -> drive
                // .withVelocityX(1).withVelocityY(0)));
                return new PathPlannerAuto("Mac Auto");
        }
}
