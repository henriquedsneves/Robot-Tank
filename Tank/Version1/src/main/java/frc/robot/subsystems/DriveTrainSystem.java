package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

public class DriveTrainSystem extends SubsystemBase {
  public DriveTrainState currentState = DriveTrainState.STOPPED;
  private Pose2d currentPose = new Pose2d(); // Armazena a pose do robô
  private ChassisSpeeds currentSpeeds = new ChassisSpeeds();

  // Motores
  private SparkMax rightMotorFront = new SparkMax(Constants.DriveTrainConstants.rightFrontMotorID, MotorType.kBrushless);
  private SparkMax rightMotorBack = new SparkMax(Constants.DriveTrainConstants.rightBackMotorID, MotorType.kBrushless);
  private SparkMax leftMotorBack = new SparkMax(Constants.DriveTrainConstants.leftBackMotorID, MotorType.kBrushless);
  private SparkMax leftMotorFront = new SparkMax(Constants.DriveTrainConstants.leftFrontMotorID, MotorType.kBrushless);

  // Giroscópio
 


  // Configuração dos motores
  private SparkMaxConfig configSparkRight = new SparkMaxConfig();
  private SparkMaxConfig configSparkLeft = new SparkMaxConfig();

  // Drive diferencial
  private DifferentialDrive differentialDrive;

  // Limitador de aceleração
  private SlewRateLimiter filter = new SlewRateLimiter(10);

  public DriveTrainSystem() {
    
    // Configuração dos motores
    configSparkRight
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);

    configSparkLeft
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);

     rightMotorFront.configure(configSparkRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      rightMotorBack.configure(configSparkRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      leftMotorFront.configure(configSparkLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      leftMotorBack.configure(configSparkLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Define a inversão dos motores manualmente para evitar conflitos com DifferentialDrive
    leftMotorFront.setInverted(false);
    leftMotorBack.setInverted(false);
    rightMotorFront.setInverted(true);
    rightMotorBack.setInverted(true);

    // Inicializa o DifferentialDrive sem usar MotorControllerGroup
    differentialDrive = new DifferentialDrive(leftMotorFront, rightMotorFront);

    setupPathplanner();
  }

  public void setupPathplanner(){
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }

    AutoBuilder.configure(
            this::getPose, // Método que retorna a pose atual do robô
            this::resetPose, // Método para resetar a odometria
            this::getRobotRelativeSpeeds, // Retorna a velocidade do robô em ChassisSpeeds
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Define a movimentação do robô
            new PPLTVController(0.02), // Controlador de trajetória para Tank Drive
            config, // Configuração do robô
            () -> {
              var alliance = DriverStation.getAlliance();
              return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this // Adiciona dependência ao subsistema
    );
  }

  @Override
  public void periodic() {
    if (currentState == DriveTrainState.STOPPED) {
        differentialDrive.stopMotor();
    }
    differentialDrive.feed(); // Mantém o DifferentialDrive atualizado
  }

  public void arcadeMode(double drive, double turn){
    differentialDrive.arcadeDrive(filter.calculate(drive), turn);
  }

  public void tankmode(double left, double right){
    differentialDrive.tankDrive(left, right);
  }

  public void stop(){
    differentialDrive.stopMotor(); 
  }

  public void SetCurrentState(DriveTrainState state){
    this.currentState = state;
  }



  
  public Pose2d getPose() {
    return currentPose; // Substituir pelo método real de odometria
  }

  public void resetPose(Pose2d newPose) {
    this.currentPose = newPose;
    // Adicione aqui o reset real da odometria se necessário
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return currentSpeeds; // Substituir pela leitura real da cinemática do robô
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    double leftSpeed = speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond;
    double rightSpeed = speeds.vxMetersPerSecond + speeds.omegaRadiansPerSecond;

    tankmode(leftSpeed, rightSpeed);
  }

  

  public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
    return new PathPlannerAuto(pathName);
  }
}
