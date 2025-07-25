// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controle;
import frc.robot.Constants.DriveTrainState;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.subsystems.DriveTrainSystem;

import java.io.File;

import com.fasterxml.jackson.core.util.RequestPayload;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class RobotContainer {
  DriveTrainSystem driveTrainSystem;
  // Aqui iniciamos o swerve

  
  boolean modoAlinhamento;

  boolean alianca = true;// true para vermelha, false para azul
  
  CommandXboxController joystick1 = new CommandXboxController(0);
  CommandXboxController joystick2 = new CommandXboxController(1);

  
  private final SendableChooser<Command> autoChooser;

  private final SendableChooser<Integer> pipelineChooser = new SendableChooser<>();
  



  public RobotContainer() {
    // Definimos o comando padrão como a tração
   

   
   

     
      
      
      autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autonomo", autoChooser);
    




   

    // Configure the trigger bQindings
    configureBindings();
    
  }

  // Função onde os eventos (triggers) são configurados
  private void configureBindings() {
    

    joystick1.button(1).whileTrue(new SetMechanismState(DriveTrainState.STOPPED));
    joystick1.button(5).whileTrue(new SetMechanismState(DriveTrainState.MID));
    joystick1.button(6).whileTrue(new SetMechanismState(DriveTrainState.FULL));

    joystick1.button(4).whileTrue(new RotatioGyro(driveTrainSystem));
    joystick1.button(2).whileTrue(new SetMechanismState(CoralState.SHOOTING)).onFalse(new SetMechanismState(CoralState.STOPPED));


    //controleXbox.y().onTrue(new InstantCommand(() -> modoAlinhamento = false));  
    //controleXbox.a().onTrue(new InstantCommand(() -> modoAlinhamento = true));

  
    // controleXbox.leftTrigger().whileTrue(new SetMechanismState(ClimberState.CLOSING)).onFalse(new SetMechanismState(ClimberState.STOPPED));


    ButtonBoxPP();

    
  }
  
  private void defaultcommands(){
    driveTrainSystem.setDefaultCommand(new DriveWithJoystick(driveTrainSystem, joystick1));
  }
  
  // funcao que retorna o autonomous
  


   

  public void ButtonBoxPP(){
    
    
    //Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);


  
      
          
              
        //    new Intake(deployerIntakeSystem),
        //    new ReShoot(deployerIntakeSystem),       
              
        //    // alinhar

              
              
  }
}