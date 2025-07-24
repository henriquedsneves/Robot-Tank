// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClimberState;
import frc.robot.Constants.Controle;
import frc.robot.Constants.ElevatorState;
import frc.robot.Constants.ModeElevator;
import frc.robot.Constants.PivoState;
import frc.robot.Constants.Tags;
import frc.robot.Constants.Timer;
import frc.robot.Constants.turboStates;
import frc.robot.commands.AutoCommands.SwerveLadoEsquerdaAuto;
import frc.robot.commands.Manipulator.GetSensor;
import frc.robot.commands.Manipulator.SetActions;
import frc.robot.commands.Manipulator.SetMechanismState;
import frc.robot.commands.OdometriaAlinhamento.AlinhamentoOdometria;
import frc.robot.commands.Reef.Alinhamento3dDireita;
import frc.robot.commands.Reef.Alinhamento3dEsquerda;
import frc.robot.commands.Reef.AlinhamentoMeio;
import frc.robot.commands.Subsystem.AlgaeGet;
import frc.robot.commands.Subsystem.AlgaeShoot;
import frc.robot.commands.Subsystem.Intake;
import frc.robot.commands.Subsystem.ReShoot;
import frc.robot.commands.Subsystem.ReShootD;
import frc.robot.commands.Subsystem.Shoot;
import frc.robot.commands.Subsystem.ShootD;
import frc.robot.commands.Subsystem.cancelElevator;
import frc.robot.commands.Subsystem.moveElevatorCommand;
import frc.robot.commands.Subsystem.moveElevatorCommandAut;
import frc.robot.commands.Subsystem.moveElevatorCommandManual;
import frc.robot.commands.swervedrive.drivebase.SwerveLadoDireita;
import frc.robot.commands.swervedrive.drivebase.SwerveLadoEsquerda;
import frc.robot.commands.swervedrive.drivebase.SwerveLadoFrente;
import frc.robot.commands.swervedrive.drivebase.Teleop;
import frc.robot.subsystems.AlgaeSystem;
import frc.robot.subsystems.ClimberSystem;
import frc.robot.subsystems.DeployerIntakeSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.PivoSystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

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
  // Aqui iniciamos o swerve

  private turboStates turbo = turboStates.marchaBaixa;
  boolean modoAlinhamento;

  boolean alianca = true;// true para vermelha, false para azul
  public final static ElevatorSystem elevatorSystem = new ElevatorSystem();
  public static final DeployerIntakeSystem deployerIntakeSystem = new DeployerIntakeSystem();
  public static final PivoSystem pivoSystem = new PivoSystem();
  public static final ClimberSystem climberSystem = new ClimberSystem();
  public static final AlgaeSystem algaeSystem = new AlgaeSystem();
  CommandXboxController joystick1 = new CommandXboxController(0);
  CommandXboxController joystick2 = new CommandXboxController(1);

  public static final Led led = new Led();
  
  private final SendableChooser<Command> autoChooser;

  private final SendableChooser<Integer> pipelineChooser = new SendableChooser<>();
  public moveElevatorCommand moveElevatorCommand;
  GetSensor getSensor;



  public RobotContainer() {
    // Definimos o comando padrão como a tração
    swerve.setDefaultCommand(new Teleop(swerve,
      () -> -MathUtil.applyDeadband(controleXbox.getLeftY() * turbo.velocidade, Constants.Controle.DEADBAND),
      () -> -MathUtil.applyDeadband(controleXbox.getLeftX() * turbo.velocidade, Constants.Controle.DEADBAND) ,
      () -> -MathUtil.applyDeadband(controleXbox.getRightX() * turbo.velocidade / 1.7, Constants.Controle.DEADBAND)));

   
   

      NamedCommands.registerCommand("L1", 
      new SetMechanismState(ElevatorState.L1).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
      NamedCommands.registerCommand("L2", 
      new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
      NamedCommands.registerCommand("L3", 
       new SetMechanismState(ElevatorState.L3).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
      NamedCommands.registerCommand("L4", 
       new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
      NamedCommands.registerCommand("Repouso", 
       new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
      NamedCommands.registerCommand("Pontuar", 
       new Shoot(deployerIntakeSystem));
      NamedCommands.registerCommand("Recolher",
       new SequentialCommandGroup(new Intake(deployerIntakeSystem)));
      
      
      autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autonomo", autoChooser);
    




    if(!Robot.isReal()){
      controleXbox.start().onTrue(Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }

    // Configure the trigger bQindings
    configureBindings();
    configurePipelineSelector();
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

    joymanual.axisMagnitudeGreaterThan(1,0.1).whileTrue(new moveElevatorCommandManual(joymanual, elevatorSystem)).onFalse(new cancelElevator(elevatorSystem));

    // joymanual.rightTrigger().whileTrue(new AlgaeGet(algaeSystem));
    // joymanual.leftTrigger().whileTrue(new AlgaeShoot(algaeSystem));

    joymanual.rightTrigger().whileTrue(new AlgaeShoot(algaeSystem));
    joymanual.leftTrigger().whileTrue(new AlgaeGet(algaeSystem));
    // joymanual.leftTrigger().onTrue(new Intake(deployerIntakeSystem));

    joymanual.button(5).onTrue(new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
    joymanual.button(1).onTrue(new ParallelCommandGroup(new AlgaeGet(algaeSystem),new SetMechanismState(ElevatorState.ALGAE1).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))))).onFalse(new AlgaeGet(algaeSystem));
    joymanual.button(4).onTrue(new ParallelCommandGroup(new AlgaeGet(algaeSystem),new SetMechanismState(ElevatorState.ALGAE2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))))).onFalse(new AlgaeGet(algaeSystem));
    joymanual.button(2).onTrue(new ParallelCommandGroup(new AlgaeGet(algaeSystem),new SetMechanismState(ElevatorState.NET).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))))).onFalse(new AlgaeGet(algaeSystem));

    joymanual.povUp().onTrue(new SetMechanismState(PivoState.OPEN));
    joymanual.povDown().onTrue(new SetMechanismState(PivoState.CLOSED));

    joymanual.button(6).whileTrue(new ReShootD(deployerIntakeSystem));

    joymanual.button(8).onTrue(new SetActions().ClimbPosition());

    controleXbox.rightTrigger().whileTrue(new SetMechanismState(ClimberState.OPENING)).onFalse(new SetMechanismState(ClimberState.STOPPED));
    // controleXbox.leftTrigger().whileTrue(new SetMechanismState(ClimberState.CLOSING)).onFalse(new SetMechanismState(ClimberState.STOPPED));

    buttonJoy.button(15).onTrue(new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
    buttonJoy.button(13).onTrue(new SetMechanismState(ElevatorState.L1).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
    buttonJoy.button(11).onTrue(new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
    buttonJoy.button(9).onTrue(new SetMechanismState(ElevatorState.L3).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));
    buttonJoy.button(6).onTrue(new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC).andThen(new moveElevatorCommand(elevatorSystem))));

    // buttonJoy.button(4).onTrue(new Intake(deployerIntakeSystem));
    buttonJoy.button(4).onTrue(new SequentialCommandGroup(new Intake(deployerIntakeSystem), new ReShoot(deployerIntakeSystem)));
    buttonJoy.button(1).whileTrue(new ShootD(deployerIntakeSystem));
    buttonJoy.button(7).onTrue(new SetActions().ClimbPosition());
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