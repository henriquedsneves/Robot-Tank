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
    
  public static final class Dimensoes {
    // Tempo de loop (sparkMax + normal = 130ms)
    public static final double LOOP_TIME = 0.13;
    // Massa do robô *DEVE SER CONFIGURADO PARA O SEU ROBÔ*
    public static final double ROBOT_MASS = 62;
    //Velocidade máxima *DEVE SER CONFIGURADO PARA O SEU ROBÔ*
    public static final double MAX_SPEED = 5
    ;
    //Posição do módulo mais longe *COLOQUE OS MESMOS VALORES DO JSON*
    private static final Translation2d FURTHEST_MODULE_POSE = new Translation2d(MAX_SPEED, LOOP_TIME);
    public static final double MAX_ANGULAR_SPEED = SwerveMath.calculateMaxAngularVelocity(MAX_SPEED, FURTHEST_MODULE_POSE.getX(), FURTHEST_MODULE_POSE.getY());

    //Posições do centro de massa *DEVE SER CONFIGURADO PARA SEU ROBÔ*
    private static final double xMass = 0;
    private static final double yMass = 0;
    private static final double zMass = .08;

    // Centro de massa do chassi
    public static final Matter CHASSIS = new Matter(new Translation3d(xMass, yMass, (zMass)), ROBOT_MASS);
   }

    // Contem a porta em que o controle está
    public static final class Controle {
      // Porta do controle
    
      
      public static final int buttonBox = 1;
      
      public static final int joyManual = 2;
      
      // Deadband do controle
      public static final double DEADBAND = 0.2;
    }


  

    

    
    public static class ElevatorConstants {
      public static int elevatorLeftMotorID = 10;
      public static int elevatorRightMotorID = 9;
  
      // public static final double PID_TOLERANCE = 0.001;
      // public static final double kP = 1.5;
      // public static final double kI = 0;
      // public static final double kD = 0;
  
      // public static final double kS = 0.14;
      // public static final double kG = 0;
      // public static final double kV = 0.0023;
      // public static final double kA = 0;
  
      public static final double ELEVATOR_MANUAL_SPEED = 0.1;
    }

    public static class DeployerIntakeConstants {
      public static int deployerLeftMotorID = 15;
      public static int deployerRightMotorID = 16;
  
      public static int deployerSensor1 = 0;
      public static int deployerSensor2 = 3;
      public static int funnelSensor = 4;
  
      public static double intakeSpeed = 0.2;
    }
  
    public static class PivoConstants {
      public static int pivoMotorID = 11;
  
      public static final double PID_TOLERANCE = 0.1;
      public static final double kP = 0.4;
      public static final double kI = 0;
      public static final double kD = 0;
  
      public static final double kS = 0;
      public static final double kG = 0;
      public static final double kV = 0;
  
      public static final double PIVO_SPEED = 0.15;
    }
  
    public static class ClimberConstants {
      public static int climberMotorID = 12;
    }

    public static class AlgaeConstants {
      public static int algaeMotorID = 13;
    }

    public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
    static {

      ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      // ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;

      ELEVATOR_CONFIG.Slot0.kG = 0.0; // Volts to overcome gravity
      ELEVATOR_CONFIG.Slot0.kS = 0.0; // Volts to overcome static friction
      // ELEVATOR_CONFIG.Slot0.kV = 0.0764; // Volts for a velocity target of 1 rps
      ELEVATOR_CONFIG.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
      ELEVATOR_CONFIG.Slot0.kA = 0; // Volts for an acceleration of 1 rps/
      ELEVATOR_CONFIG.Slot0.kP = 0.85;
      ELEVATOR_CONFIG.Slot0.kI = 0;
      ELEVATOR_CONFIG.Slot0.kD = 0;
      // ELEVATOR_CONFIG.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

      // ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      // ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorState.L4.position;
      // ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      // ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorState.CORAL.position;

      ELEVATOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 0;
      ELEVATOR_CONFIG.MotionMagic.MotionMagicAcceleration = 0;
      ELEVATOR_CONFIG.MotionMagic.MotionMagicExpo_kV = 0;
      ELEVATOR_CONFIG.MotionMagic.MotionMagicExpo_kA = 0.015;

      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 0;
      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 1;
    }


    public static TalonFXConfiguration CLIMBER_CONFIG = new TalonFXConfiguration();
    static {
      
      CLIMBER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      CLIMBER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 60;
      CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLimit = 85;

      CLIMBER_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      CLIMBER_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 200;
      CLIMBER_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      CLIMBER_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    }
  
    //States+
  
    public static enum ElevatorState {
      CORAL(1.32), L1(18), L2(32), L3(54), L4(86.5625), ALGAE1(20), ALGAE2(41), NET(86.5625);
      public final double position; 
      
      private ElevatorState(double position){
        this.position = position;
      }
    }
  
    //States+
  
    public static enum ModeElevator {
      MANUAL(true), AUTOMATIC(false);
      public final boolean mode;
      
      
      private ModeElevator(Boolean mode){
        this.mode = mode;
      }
    }
  
    public static enum IntakeState {
      STOPPED(false), OPERATION(true);
      public final boolean operation;
      
      private IntakeState(Boolean operation){
        this.operation = operation;
      }
    }
  
    public static enum DeployerState {
      RESHOOTING(-0.21), STOPPED(0), SHOOTING(0.22);
      public final double speed; 
      
      private DeployerState(double speed){
        this.speed = speed;
      }
    }
  
    public static enum PivoState {
      CLOSED(0.962), OPEN(6);
      public final double position;
      
      private PivoState(double position){
        this.position = position;
      }
    }
  
    public static enum ClimberState {
      CLOSING(-0.7), STOPPED(0), OPENING(0.7);
      public final double speed;
      
      private ClimberState(double speed){
        this.speed = speed;
      }
    }

    public static enum AlgaeState {
      GET(-1), STOPPED(0), SHOOT(1);
      public final double speed;
      
      private AlgaeState(double speed){
        this.speed = speed;
      }
    }

    
  public static enum DriveTrainState {
    STOPPED(0), MID(0.3), FULL(0.7);
    public final double velocity;
    
    private DriveTrainState(double velocity){
      this.velocity = velocity;
    }
  }
  }
  
