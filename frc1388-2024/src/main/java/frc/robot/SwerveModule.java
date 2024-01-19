package frc.robot;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    
    private final TalonFX m_driveMotor;
    

    private final CANcoder m_cancoder;
    private final double m_encoderOffset;

    private final TalonFX m_rotationMotor;
    private final PIDController m_rotationPID;

    public SwerveModule(TalonFX driveMotor, TalonFX rotationMotor, CANcoder cancoder, double encoderOffset) {
        m_driveMotor = driveMotor;
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
        Slot0Configs driveConfig = new Slot0Configs();
        driveConfig.kP = Constants.SwerveModuleConstants.kDriveMotorP;
        driveConfig.kI = Constants.SwerveModuleConstants.kDriveMotorI;
        driveConfig.kD = Constants.SwerveModuleConstants.kDriveMotorD;
        m_driveMotor.getConfigurator().apply(driveConfig);

        m_rotationMotor = rotationMotor;
        m_rotationMotor.setNeutralMode(NeutralModeValue.Brake);
        m_rotationMotor.setInverted(true);

        m_encoderOffset = encoderOffset;

        m_rotationPID = new PIDController(Constants.SwerveModuleConstants.kRotationP,
                                          Constants.SwerveModuleConstants.kRotationI,
                                          Constants.SwerveModuleConstants.kRotationD);
        m_rotationPID.setTolerance(Constants.SwerveModuleConstants.kRotationTolerance);
        m_rotationPID.enableContinuousInput(0, 360);

        m_cancoder = cancoder;
        MagnetSensorConfigs cancoderConfig = new MagnetSensorConfigs();
        cancoderConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoderConfig.MagnetOffset = 0;
        cancoderConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        m_cancoder.getConfigurator().apply(cancoderConfig);
    }
    
    public void setSwerveModuleStates(SwerveModuleState inputState) {
        Rotation2d rotation = new Rotation2d(Math.toRadians(getRotationAngle()));
        SwerveModuleState swerveModuleState = SwerveModuleState.optimize(inputState, rotation);
        setDriveSpeed(swerveModuleState.speedMetersPerSecond);
        setRotationPosition(swerveModuleState.angle.getDegrees());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            -m_driveMotor.getPosition().getValue() * Constants.SwerveModuleConstants.DIST_PER_TICK,
            new Rotation2d(Math.toRadians(getRotationAngle()))
        );
    }

    public void setDriveSpeed(double inputSpeed) {
        // because the robot's max speed is 3 m/s, deviding the speed by 3 results in a power [-1, 1] we can set the motor to 
        m_driveMotor.set(inputSpeed / Constants.DriveTrainConstants.ROBOT_MAX_SPEED); // probably should be done outside of swerve module
    }

    public void setRotationPosition(double angle) {
        m_rotationMotor.set(m_rotationPID.calculate(getRotationAngle(), angle));
    }

    public double getRotationAngle() {
        return (m_cancoder.getAbsolutePosition().getValue() * 360 - m_encoderOffset + 36000) % 360 - 90; // math should be reviewed and ask what they do for constants
    }

    public void periodic() {
        // if the DriveTrain subsystem periodic is calling this, this method acts as a periodic
    }
}
