// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;

/**
 * Classe de constantes
 */
public final class Constants {
  public static class DriveTrainConstants {
    public static int rightFrontMotorID = 14;
    public static int rightBackMotorID = 10;
    public static int leftFrontMotorID = 11;
    public static int leftBackMotorID = 9;
  }
  public static class ControlsJoystick {
    public static int leftMotors = 5;
    public static int rightMotors = 1;
  }
  // Aqui temos várias constantes referentes as demais áreas do robô
    
  
    // Centro de massa do chassi
    

    // Contem a porta em que o controle está
    public static final class Controle {
      // Porta do controle
    
      
      public static final int buttonBox = 1;
      
      public static final int joyManual = 2;
      
      // Deadband do controle
      public static final double DEADBAND = 0.2;
    
      public static enum DriveTrainState {
        STOPPED(0), MID(0.3), FULL(0.7);
        public final double velocity;
        
        private DriveTrainState(double velocity){
          this.velocity = velocity;
        }
      }}


  

    

   
    

    
     

      // ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;

      // ELEVATOR_CONFIG.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

      // ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      // ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorState.L4.position;
      // ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      // ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorState.CORAL.position;



    //States+
  
  
  
    //States+
  
   
  
  

  

    
  public static enum DriveTrainState {
    STOPPED(0), MID(0.3), FULL(0.7);
    public final double velocity;
    
    private DriveTrainState(double velocity){
      this.velocity = velocity;
    }
  }
}
  
  
