// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransitionConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.vision.Limelight;

public class IntakeTransitionCommand extends Command {

  public enum IntakeTransState {
    DEPLOYING, RETRACTING, TRANSITION, DONE
  }
  private IntakeTransState m_state;
  private IntakeTransState m_initialState;


  private final boolean m_pullToTransition;

  private final Limelight m_Limelight;

  IntakeSubsystem m_intakeSubsystem;
  private int m_ticksNoteIsDetectedInIntake = 0;
  TransitionSubsystem m_transitionSubsystem;

  /** Creates a new IntakeTransitionCommand. */
  public IntakeTransitionCommand(IntakeTransState initialState, boolean pullToTransition, IntakeSubsystem intakeSubsystem, TransitionSubsystem transitionSubsystem, Limelight limelight) {

    m_initialState = initialState;

    m_pullToTransition = pullToTransition;

    m_Limelight = limelight;

    m_intakeSubsystem = intakeSubsystem;
    m_transitionSubsystem = transitionSubsystem;

    addRequirements(m_intakeSubsystem, m_transitionSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state = m_initialState;
    System.out.println(m_state.name());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_state) {


      case DEPLOYING:
        m_intakeSubsystem.setLifterMotor(IntakeConstants.LIFTER_MOTOR_SPEED_DOWN);
        if (m_intakeSubsystem.isNoteDetected()) {
          m_ticksNoteIsDetectedInIntake++;
          m_intakeSubsystem.setRollerMotor(0);
        } else {
          m_ticksNoteIsDetectedInIntake = 0;
          m_intakeSubsystem.setRollerMotor(IntakeConstants.ROLLER_MOTOR_SPEED_IN_INTAKING);
        }
        if (m_ticksNoteIsDetectedInIntake > IntakeConstants.TICKS_BEFORE_RETRACTING_INTAKE) {
          m_state = IntakeTransState.RETRACTING;
              System.out.println(m_state.name());

        }
        break;


      case RETRACTING:
      m_Limelight.setLimelightLEDsOn(true);
        m_intakeSubsystem.setLifterMotor(IntakeConstants.LIFTER_MOTOR_SPEED_UP);
        m_intakeSubsystem.setRollerMotor(0);
        if (m_intakeSubsystem.atUpperLimit()) {
          if (m_pullToTransition) {
            m_state = IntakeTransState.TRANSITION;
                System.out.println(m_state.name());

          } else {
            m_state = IntakeTransState.DONE;
                System.out.println(m_state.name());

          }
        }
        break;
      
      case TRANSITION:
        m_Limelight.setLimelightLEDsOn(false);
        m_transitionSubsystem.set(TransitionConstants.TRANSITION_MOTOR_POWER_IN);
        m_intakeSubsystem.setRollerMotor(IntakeConstants.ROLLER_MOTOR_SPEED_IN_TRANSITION);
        if (m_transitionSubsystem.isNoteDetected()) {
          m_state = IntakeTransState.DONE;
              System.out.println(m_state.name());

        }
        break;


      default:
        m_state = IntakeTransState.DONE;
            System.out.println(m_state.name());

        break;
    }
    SmartDashboard.putString("intake transition state", m_state.name());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Limelight.setLimelightLEDsOn(false);
    m_intakeSubsystem.setLifterMotor(0);
    m_intakeSubsystem.setRollerMotor(0);
    m_transitionSubsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_state == IntakeTransState.DONE;
  }
}
