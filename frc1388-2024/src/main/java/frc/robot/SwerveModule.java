package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    
    private final TalonFX m_driveMotor;
    private final double DIST_PER_TICK = (1.0 / 6.75) * (0.3192);

    private final CANcoder m_cancoder;
    private final double ENCODER_OFFSET;

    private final TalonFX m_rotationMotor;
    private final PIDController m_rotationPID;

    public SwerveModule(TalonFX driveMotor, TalonFX rotationMotor, CANcoder cancoder, double encoderOffset) {
        m_driveMotor = driveMotor;
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
        Slot0Configs driveConfig = new Slot0Configs();
        driveConfig.kP = 0.001;
        driveConfig.kI = 0;
        driveConfig.kD = 0;
        m_driveMotor.getConfigurator().apply(driveConfig);

        m_rotationMotor = rotationMotor;
        m_rotationMotor.setNeutralMode(NeutralModeValue.Brake);
        m_rotationMotor.setInverted(true);

        ENCODER_OFFSET = encoderOffset;

        m_rotationPID = new PIDController(0.007, 0, 0);
        m_rotationPID.setTolerance(5);
        m_rotationPID.enableContinuousInput(0, 360);

        m_cancoder = cancoder;



    }
    
    public void setSwerveModuleStates(SwerveModuleState inputState) {
        Rotation2d rotation = new Rotation2d(Math.toRadians(getRotationAngle()));
        SwerveModuleState swerveModuleState = SwerveModuleState.optimize(inputState, rotation);
        setDriveSpeed(swerveModuleState.speedMetersPerSecond);
        setRotationPosition(swerveModuleState.angle.getDegrees());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            -m_driveMotor.getPosition().getValue() * DIST_PER_TICK,
            new Rotation2d(Math.toRadians(getRotationAngle()))
        );
    }

    public void setDriveSpeed(double inputSpeed) {
        // because the robot's max speed is 3 m/s, deviding the speed by 3 results in a power [.1, 1] we can set the motor to 
        m_driveMotor.set(inputSpeed / 3.0);
    }

    public void setRotationPosition(double angle) {
        m_rotationMotor.set(m_rotationPID.calculate(getRotationAngle(), angle));
    }

    public double getRotationAngle() {
        return (m_cancoder.getAbsolutePosition().getValue() * 360 - ENCODER_OFFSET + 36000) % 360 - 90;
    }

    public void periodic() {
        // if the DriveTrain subsystem periodic is calling this, this method acts as a periodic
    }
}
