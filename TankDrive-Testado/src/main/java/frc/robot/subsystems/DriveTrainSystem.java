// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;



import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSystem extends SubsystemBase {

  SparkMax rightMotorFront = new SparkMax(Constants.DriveTrainConstants.rightFrontMotorID, MotorType.kBrushed);
  SparkMax rightMotorBack = new SparkMax(Constants.DriveTrainConstants.rightBackMotorID, MotorType.kBrushed);
  SparkMax leftMotorBack = new SparkMax(Constants.DriveTrainConstants.leftBackMotorID, MotorType.kBrushed);
  SparkMax leftMotorFront = new SparkMax(Constants.DriveTrainConstants.leftFrontMotorID, MotorType.kBrushed);



  
  MotorControllerGroup leftMotorControllerGroup = new MotorControllerGroup(leftMotorFront, leftMotorBack);
  MotorControllerGroup rightMotorControllerGroup = new MotorControllerGroup(rightMotorFront, rightMotorBack);
  
  DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorControllerGroup, rightMotorControllerGroup);
  
  SparkMaxConfig configSparkRight = new SparkMaxConfig();
  SparkMaxConfig configSparkLeft = new SparkMaxConfig();
  

  public DriveTrainSystem() {
   
    configSparkRight
    .inverted(false )
    .idleMode(IdleMode.kBrake);

  configSparkLeft
    .inverted(false)
    .idleMode(IdleMode.kBrake);

  configSparkRight.smartCurrentLimit(60);
  configSparkLeft.smartCurrentLimit(60);

  rightMotorFront.configure(configSparkRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  rightMotorBack.configure(configSparkRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  leftMotorFront.configure(configSparkLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  leftMotorBack.configure(configSparkLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

// rightMotorFront.configure(configSparkMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
// leftMotorBack.configure(configSparkMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
// leftMotorFront.configure(configSparkMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightMotorControllerGroup.setInverted(true);
    leftMotorControllerGroup.setInverted(false);

    
    

  
  

  
  


  }


  @Override
  public void periodic() {
    
  }

  public void arcadeMode(double drive, double turn){
    differentialDrive.arcadeDrive(drive, turn);
  }

  public void tankmode(double left, double right){
    differentialDrive.tankDrive(left, right);
  }
  public void stop(){
    differentialDrive.stopMotor(); 
  }

}
