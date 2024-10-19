// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveMaintainHeading extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private DoubleSupplier vX;
  private DoubleSupplier vY;
  private DoubleSupplier vOmega;

  private Optional<Rotation2d> headingSetpoint;
  private double mJoystickLastTouched = -1;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(CommandSwerveDrivetrain.MaxSpeed * 0.1).withRotationalDeadband(CommandSwerveDrivetrain.MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private SwerveRequest.FieldCentricFacingAngle driveWithHeading = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(CommandSwerveDrivetrain.MaxSpeed * 0.1)
    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  /** Creates a new DriveMaintainHeading. */
  public DriveMaintainHeading(CommandSwerveDrivetrain drivetrain, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier vOmega) {
    this.drivetrain = drivetrain;
    this.vX = vX;
    this.vY = vY;
    this.vOmega = vOmega;

    driveWithHeading.HeadingController.setPID(DriveConstants.kHeadingP, DriveConstants.kHeadingI, DriveConstants.kHeadingD);
    driveWithHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
    setName("Swerve with Maintain Heading");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = vY.getAsDouble() * CommandSwerveDrivetrain.MaxSpeed;
    double strafe = vX.getAsDouble() * CommandSwerveDrivetrain.MaxSpeed;
    double turnFieldFrame = vOmega.getAsDouble();
    double throttleFieldFrame =0;
    double strafeFieldFrame = 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
