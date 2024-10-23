// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveMaintainingHeading;
import frc.robot.commons.GremlinPS4Controller;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsytem;
import frc.robot.vision.GremlinApriltagVision;

public class RobotContainer {
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final GremlinPS4Controller joystick = new GremlinPS4Controller(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public final ArmSubsystem arm = new ArmSubsystem();
  public final ShooterSubsytem shooter = new ShooterSubsytem();
  public final IntakeSubsystem intake = new IntakeSubsystem();

  /*Vision System */
  public final GremlinApriltagVision apriltagVision = new GremlinApriltagVision(
    VisionConstants.cameras, 
    () -> drivetrain.getState().Pose, 
    updates -> drivetrain.addVisionMeasurements(updates));

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  @SuppressWarnings("unused")
  private Command runAuto = drivetrain.getAutoPath("Tests");
  private Command seedMiddlePosition = drivetrain.runOnce(() -> drivetrain.seedFieldRelative(PathPlannerAuto.getStaringPoseFromAutoFile("Start Middle")));

  private final Telemetry logger = new Telemetry(DriveConstants.TrueMaxSpeed);


  /*Commands */
  private final Command defaultShotCommand = shooter.setRevving()
      .alongWith(arm.goToDefaultShot())
      .alongWith(new InstantCommand(() -> shooter.setDefaultShot(true)));
  
  private final DriveMaintainingHeading driveCommand = new DriveMaintainingHeading(
    drivetrain, joystick::getLeftYReversed, joystick::getLeftXReversed, joystick::getRightXReversed);

  private void configureBindings() {
    drivetrain.setDefaultCommand(driveCommand);

    joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.circle().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    //joystick.L2().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.povDown().OnPressTwice(drivetrain.toggleSlowMode(), drivetrain.toggleFastMode());

    /*Bindings to set State of Shooter*/
    //Should have shooters rev, Essentially sets everything up for the later trigger
    // joystick.triangle().toggleOnTrueNoInterrupt(
    //   shooter.setRevving()
    //   .alongWith(arm.setAngleFromDistance(() -> drivetrain.getSpeakerDistanceMoving()))
    //   .alongWith( 
    //     drivetrain.aimAtSpeakerMoving(
    //       () -> -joystick.getLeftY() * DriveConstants.appliedMaxSpeed, 
    //       ()-> -joystick.getLeftX() * DriveConstants.appliedMaxSpeed))
    //   .finallyDo((interrupted) -> {
    //     if(!interrupted){
    //         shooter.coastShootersAndIdleRunnable();
    //     }
    //   })
    // ); 

    joystick.triangle().toggleOnTrueNoInterrupt(
      shooter.setRevving()
      .alongWith(arm.setAngleFromDistance(() -> drivetrain.getSpeakerDistanceMoving()))
      // .alongWith(drivetrain.aimAtSpeakerMoving(
      //   () -> -joystick.getLeftY() * DriveConstants.appliedMaxSpeed, 
      //   ()-> -joystick.getLeftX() * DriveConstants.appliedMaxSpeed))
      .finallyDo((interrupted) -> {
        if(!interrupted){
          shooter.coastShootersAndIdleRunnable();
          shooter.stopFeedRunnable();
        }
      }));

    joystick.L2().toggleOnTrueNoInterrupt( //DEFAULT SHOT COMMAND TODO: ADD TURN TO ANGLE
      defaultShotCommand.finallyDo((interrupted) -> {
        if(!interrupted){
          shooter.coastShootersAndIdleRunnable();
          shooter.stopFeedRunnable();
        }
      })
    );

    /*Triggers to deal with State of Shooter */
    (shooter.isRevving.or(shooter.isShooting))
      .and(shooter.hasNoteBack)
      .and(shooter.atSpeed)
      .and(arm.atTarget)
      .and(drivetrain.withinRange.or(shooter.defaultShotTrigger)) //TODO: add a check that a main "shooter" camera sees the target
      .whileTrue(
        shooter.setShooting().andThen(shooter.feedNote())); //TODO: add a cancel so we go back to normal rotation maybe

    shooter.isIdle
      .onTrue(
        shooter.coastShootersAndIdle()
        .andThen(arm.goToMin()));
    
    //Once the note is gone we're done shooting so we go idle and coast the shooters
    shooter.isShooting.and(shooter.hasNoteBack.negate()).whileTrue(
      shooter.coastShootersAndIdle()
      .andThen(shooter.stopFeed())
      .andThen(Commands.print("Stopped After shot")));

     /*Intake Bindings */ 
    joystick.R1().and(shooter.hasNoteBack.negate()).toggleOnTrue( 
        (arm.goToIntake()
          .andThen(Commands.waitUntil(arm.atIntake))
          .andThen(intake.setIntakingState())
          .andThen(intake.runIntakeMotor()
            .finallyDo(() -> intake.stopRunnable())))
        .alongWith(
          shooter.startIntaking()
            .finallyDo(() -> shooter.stopIntaking())
        )
    );

    /*Trigger to deal with Intake state */
    intake.isIntaking
      .and(shooter.hasNoteBack)
      .onTrue(
        (intake.stop()
        .alongWith(new InstantCommand(() -> shooter.stopIntaking())))
        .andThen(shooter.runBackSlow().until(shooter.hasNoteFront))
        .andThen(shooter.runForwardSlow().until(shooter.hasNoteFront.negate()))
      );
  
    joystick.L1().OnPressTwice(
      drivetrain.driveFacingAngleCommand(
        () -> -joystick.getLeftY() * DriveConstants.appliedMaxSpeed, 
        ()-> -joystick.getLeftX() * DriveConstants.appliedMaxSpeed, 
        () -> FieldConstants.ampAngle).alongWith(
      shooter.setAmp().andThen(arm.goToAmp())), 

      shooter.ampShot().until(shooter.hasNoteBack.negate().and(shooter.hasNoteFront.negate()))
        .andThen(shooter.stopFeed())
        .andThen(drivetrain.driveBackFromAmp())
        .andThen(shooter.coastShootersAndIdle())
    );

    joystick.R2().onTrue(new InstantCommand(() -> arm.findZero()));    

    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    joystick.share().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.share().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    /* Bindings for arm characterization */
    // joystick.share().and(joystick.triangle()).onTrue(arm.sysIdDynamicForward());
    // joystick.share().and(joystick.square()).onTrue(arm.sysIdDynamicReverse());
    // joystick.options().and(joystick.triangle()).onTrue(arm.sysIdQuasistaticForward());
    // joystick.options().and(joystick.square()).onTrue(arm.sysIdQuasistaticReverse());

    /* Bindings for shooter characterization */
    // joystick.share().and(joystick.triangle()).onTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // joystick.share().and(joystick.square()).onTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // joystick.options().and(joystick.triangle()).onTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // joystick.options().and(joystick.square()).onTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return seedMiddlePosition;

  }
}
