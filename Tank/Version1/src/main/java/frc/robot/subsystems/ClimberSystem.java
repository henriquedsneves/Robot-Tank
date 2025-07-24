// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberState;

import frc.robot.Constants.PivoState;

public class ClimberSystem extends SubsystemBase {
  private TalonFX climberMotor = new TalonFX(Constants.ClimberConstants.climberMotorID);

  public ClimberState stateClimber = ClimberState.STOPPED;

  public ClimberSystem() {
    climberMotor.getConfigurator().apply(Constants.CLIMBER_CONFIG);
  }

  public void periodic() {
    SmartDashboard.putNumber("Encoder Climber", climberMotor.getPosition().getValueAsDouble());

    if(stateClimber == ClimberState.OPENING){
        climberMotor.set(stateClimber.speed);
    } else if(stateClimber == ClimberState.CLOSING){
        climberMotor.set(stateClimber.speed);
    } else{
        climberMotor.stopMotor();
    }
  }

  public void SetCurrentStateClimber(ClimberState state){
    this.stateClimber = state;
  }
}
