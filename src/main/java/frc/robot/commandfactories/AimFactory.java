// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandfactories;

import static frc.robot.constants.ShooterConstants.maxAccel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsytem;

/** Add your docs here. */
public class AimFactory {
    public static Command aimShooterAndShoot(ShooterSubsytem shooter, ArmSubsystem arm, DoubleSupplier setpoint){
        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                shooter.setRevving().alongWith(arm.goToAngle(setpoint)),
                Commands.waitUntil(shooter.atSpeed.and(arm.atTarget))
            ),
            shooter.setShooting(),
            shooter.feedNote().until(shooter.hasNoteBack.negate()),
            shooter.stopFeed().andThen(shooter.coastShootersAndIdle())
        );
    }

    public static Command stowAndCoast(ShooterSubsytem shooter, ArmSubsystem arm){
        return new SequentialCommandGroup(
            shooter.stopFeed(),
            shooter.coastShootersAndIdle(),
            arm.goToIntake()
        );
    }
}
