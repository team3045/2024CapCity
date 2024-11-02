// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.GremlinLogger;

public class OrchestraSubsytem extends SubsystemBase {
  private Orchestra mOrchestra = new Orchestra();
  /** Creates a new OrchestraSubsytem. */
  public OrchestraSubsytem(TalonFX... motors) {
    for(TalonFX motor : motors){
      mOrchestra.addInstrument(motor);
      motor.getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(true));
    }

    var status = mOrchestra.loadMusic("fire.chrp");
    if(!status.isOK()){
      GremlinLogger.logFault(status.getDescription());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command playMusic(){
    return this.run( () -> {
      mOrchestra.play();
    });
  }

  public Command stopMusic(){
    return this.run(() -> {
      mOrchestra.stop();
    });
  }
}
