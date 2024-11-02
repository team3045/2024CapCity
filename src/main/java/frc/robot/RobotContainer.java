// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
    drivetrain::addVisionMeasurements);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  @SuppressWarnings("unused")
  //private Command runAuto = drivetrain.getAutoPath("Tests");
  private Command seedMiddlePosition = drivetrain.runOnce(() -> drivetrain.seedFieldRelative(PathPlannerAuto.getStaringPoseFromAutoFile("Start Middle")));

  private final Telemetry logger = new Telemetry(DriveConstants.TrueMaxSpeed);


  /*Commands */
  private final Command defaultShotCommand = shooter.setRevving()
      .alongWith(arm.goToDefaultShot())
      .alongWith(new InstantCommand(() -> shooter.setDefaultShot(true)));

  private final Command autoAimShotCommand = shooter.setRevving()
    .alongWith(arm.setAngleFromDistance(() -> drivetrain.getSpeakerDistanceMoving()))
    .alongWith(drivetrain.aimAtSpeakerMoving(
      () -> -joystick.getLeftY() * DriveConstants.appliedMaxSpeed, 
      ()-> -joystick.getLeftX() * DriveConstants.appliedMaxSpeed))
    .finallyDo((interrupted) -> {
      if(!interrupted){
        shooter.coastShootersAndIdleRunnable();
        shooter.stopFeedRunnable();
      }
    });

  private final Command passCommand = shooter.setRevving()
    .alongWith(arm.goToPass())
    .alongWith(new InstantCommand(() -> shooter.setDefaultShot(true)))
    .alongWith(drivetrain.aimAtSpeakerMoving(
        () -> -joystick.getLeftY() * DriveConstants.appliedMaxSpeed, 
        ()-> -joystick.getLeftX() * DriveConstants.appliedMaxSpeed))
    .finallyDo((interrupted) -> {
        if(!interrupted){
            shooter.coastShootersAndIdleRunnable();
            shooter.stopFeedRunnable();
        }
    });
  
  private final DriveMaintainingHeading driveCommand = new DriveMaintainingHeading(
    drivetrain, joystick::getLeftYReversed, joystick::getLeftXReversed, joystick::getRightXReversed, true);

  private void configureBindings() {
    drivetrain.setDefaultCommand(
      drivetrain.getDriveCommand(
        () -> -joystick.getLeftY() * DriveConstants.appliedMaxSpeed, 
        () -> -joystick.getLeftX() * DriveConstants.appliedMaxSpeed, 
        () -> -joystick.getRightX() * DriveConstants.appliedMaxAngularRate)
    );

    joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));

    // reset the field-centric heading on left bumper press
    drivetrain.registerTelemetry(logger::telemeterize);

    //joystick.R2().OnPressTwice(drivetrain.toggleSlowMode(), drivetrain.toggleFastMode());
    joystick.square().onTrue(new InstantCommand(() -> drivetrain.seedFieldRelative()));

    /*Bindings to set State of Shooter*/
    //Should have shooters rev, Essentially sets everything up for the later trigger
    joystick.L1().toggleOnTrueNoInterrupt(autoAimShotCommand);

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
      .and(() -> DriverStation.isTeleop())
      .and(shooter.hasNoteBack.or(shooter.defaultShotTrigger))
      .and(shooter.atSpeed)
      .and(arm.atTarget)
      .and(drivetrain.withinRange.or(shooter.defaultShotTrigger)) //TODO: add a check that a main "shooter" camera sees the target
      .whileTrue(
        shooter.setShooting().andThen(shooter.feedNote())); //TODO: add a cancel so we go back to normal rotation maybe

    shooter.isIdle
    .and(() -> DriverStation.isTeleop())
      .onTrue(
        shooter.coastShootersAndIdle()
        .andThen(arm.goToMin()));
    
    //Once the note is gone we're done shooting so we go idle and coast the shooters
    shooter.isShooting
    .and(() -> DriverStation.isTeleop())
    .and(shooter.hasNoteBack.negate()).whileTrue(
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
                    .finallyDo(() -> shooter.stopIntaking())));

    /*Trigger to deal with Intake state */
    intake.isIntaking
      .and(() -> DriverStation.isTeleop())
      .and(shooter.hasNoteBack)
      .onTrue(
        (intake.stop()
        .alongWith(new InstantCommand(() -> shooter.stopIntaking())))
        .andThen(shooter.runBackSlow().until(shooter.hasNoteFront))
        .andThen(shooter.runForwardSlow().until(shooter.hasNoteFront.negate()))
      );

    joystick.share().toggleOnTrue(intake.reverseIntakeMotor().finallyDo(() -> intake.stopRunnable()));
    joystick.options().toggleOnTrueNoInterrupt(passCommand);
  
    joystick.triangle().OnPressTwice(
      drivetrain.driveFacingAngleCommand(
        () -> -joystick.getLeftY() * DriveConstants.appliedMaxSpeed, 
        ()-> -joystick.getLeftX() * DriveConstants.appliedMaxSpeed, 
        () -> FieldConstants.ampAngle).alongWith(
      shooter.setAmp().andThen(arm.goToAmp())), 

      (shooter.ampShot().until(shooter.hasNoteBack.negate().and(shooter.hasNoteFront.negate())))
        .alongWith(Commands.waitSeconds(0.5))
        .andThen(shooter.stopFeed())
        .andThen(drivetrain.driveBackFromAmp())
        .andThen(shooter.coastShootersAndIdle())
    );

    joystick.povDown().onTrue(new InstantCommand(() -> arm.findZero()));    

    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    // joystick.share().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.share().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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
    registerNamedCommands();
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return Robot.autoChooser.getSelected();
  }

  /*Auto Commands */
  public Command intakeAuto() {
    return arm.goToIntake().andThen(intake.runIntakeMotor().alongWith(shooter.startIntaking())).until(shooter.hasNoteBack);
  } 

  public Command stopIntake() { 
    return intake.stop()
        .alongWith(new InstantCommand(() -> shooter.stopIntaking()));
  }
  public Command aimAndRev() { 
   return shooter.setRevving().alongWith(
      arm.setAngleFromDistance(() -> drivetrain.getSpeakerDistanceMoving()))
    .until(arm.atTarget.and(shooter.atSpeed));
  }

  public Command stopShooter(){ 
    return shooter.stopFeed().andThen(shooter.coastShootersAndIdle());
  }

  public Command shootSequence() { 
    return shooter.setShooting().andThen(shooter.feedNote()).until(shooter.hasNoteBack.negate());
  }
  public Command stopAndReset() { 
    return arm.goToIntake().andThen(stopShooter());
  }

  public void registerNamedCommands(){
    NamedCommands.registerCommand("intakeAndStop", intakeAuto().andThen(stopIntake()));
    NamedCommands.registerCommand("preload", 
      shooter.setRevving().alongWith(arm.goToDefaultShot()).until(arm.atTarget.and(arm.atIntake.negate()))
      .andThen(shootSequence()).andThen(stopAndReset()));
    NamedCommands.registerCommand("aim", aimAndRev());
    NamedCommands.registerCommand("shootSequence", shootSequence().andThen(stopAndReset()));
    NamedCommands.registerCommand("Test Print", Commands.print("Test").repeatedly());
    NamedCommands.registerCommand("IntakeAimShoot", 
      intakeAuto().andThen(stopIntake()).andThen(aimAndRev())
      .andThen(shootSequence().andThen(stopAndReset())).withTimeout(5));
    NamedCommands.registerCommand("aimAndShoot", aimAndRev()
      .andThen(shootSequence().andThen(stopAndReset())));
  } 
}
