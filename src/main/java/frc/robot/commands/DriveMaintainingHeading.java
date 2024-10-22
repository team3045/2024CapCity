// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.constants.DriveConstants.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveMaintainingHeading extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private DoubleSupplier vX;
  private DoubleSupplier vY;
  private DoubleSupplier vOmega;
  private Optional<Rotation2d> headingSetpoint;
  private double mJoystickLastTouched = -1;

  public static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  public static double MaxAngularRate = TrueMaxAngularRate;

  private SwerveRequest.FieldCentric driveNoHeading = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * XYDeadband) // Add a 10% deadband in open loop
            .withRotationalDeadband(MaxAngularRate * RotationDeadband)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
  private SwerveRequest.FieldCentricFacingAngle driveMaintainAngle = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed * XYDeadband).withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** Creates a new DriveMaintainingHeading. */
  public DriveMaintainingHeading(CommandSwerveDrivetrain drivetrain, 
    DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier vOmega) {
  
      this.drivetrain = drivetrain;
      this.vX = vX;
      this.vY = vY;
      this.vOmega = vOmega; 

      addRequirements(drivetrain);
      setName("Swerve Maintaining Heading");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    headingSetpoint = Optional.empty();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = vX.getAsDouble() * MaxSpeed;
    double ySpeed = vY.getAsDouble() * MaxSpeed;
    double rotSpeedFieldFrame = vOmega.getAsDouble();
    double xFieldFrame = Robot.isRedAlliance() ? -xSpeed : xSpeed;
    double yFieldFrame = Robot.isRedAlliance() ? -ySpeed : ySpeed;

    if(Math.abs(rotSpeedFieldFrame) > RotationDeadband)
      mJoystickLastTouched = Timer.getFPGATimestamp();

    if(Math.abs(rotSpeedFieldFrame) > RotationDeadband 
      || (MathUtil.isNear(mJoystickLastTouched, Timer.getFPGATimestamp(), 0.25)
        && drivetrain.getState().speeds.omegaRadiansPerSecond > Units.degreesToRadians(10))){

        rotSpeedFieldFrame = rotSpeedFieldFrame * MaxAngularRate;
        drivetrain.setControl(driveNoHeading
          .withVelocityX(xFieldFrame)
          .withVelocityY(yFieldFrame)
          .withRotationalRate(rotSpeedFieldFrame));
    } else {
      if(headingSetpoint.isEmpty())
        headingSetpoint = Optional.of(drivetrain.getState().Pose.getRotation());

      drivetrain.setControl(driveMaintainAngle
        .withVelocityX(xFieldFrame)
        .withVelocityY(yFieldFrame)
        .withTargetDirection(headingSetpoint.get()));
    }

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
