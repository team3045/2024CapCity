// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.ArmAngles;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private double disableStartTime = 0;

  private static boolean hasGottenTeamColor = false;
  private static Alliance allianceColor = Alliance.Blue;

  public static SendableChooser<Command> autoChooser;


  @Override
  public void robotInit() {
    ArmAngles.initLookuptable();
    m_robotContainer = new RobotContainer();

    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);

     autoChooser = AutoBuilder.buildAutoChooser("Start Middle");

    SmartDashboard.putData("Auto Choose", autoChooser);
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    checkAllianceColor();
  }

  @Override
  public void disabledInit() {
    disableStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void disabledPeriodic() {
    if(Timer.getFPGATimestamp() - disableStartTime > 10){
      m_robotContainer.arm.setCoast();
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
  }

  @Override
  public void simulationInit(){
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
  }

  public static boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
  }

  public static void checkAllianceColor(){
     /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasGottenTeamColor || DriverStation.isDisabled()) {
          DriverStation.getAlliance().ifPresent((alliance) -> {
              allianceColor = alliance;
          });
      }
  }
}
