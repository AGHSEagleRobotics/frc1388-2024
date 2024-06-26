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
    private double m_encoderOffset;

    private final TalonFX m_rotationMotor;
    private final PIDController m_rotationPID;

    public SwerveModule(TalonFX driveMotor, TalonFX rotationMotor, CANcoder cancoder, double encoderOffset) {
        m_driveMotor = driveMotor;
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
        // PID configs
        Slot0Configs driveConfig = new Slot0Configs();
        driveConfig.kP = Constants.SwerveModuleConstants.DRIVE_MOTOR_P;
        driveConfig.kI = Constants.SwerveModuleConstants.DRIVE_MOTOR_I;
        driveConfig.kD = Constants.SwerveModuleConstants.DRIVE_MOTOR_D;
        m_driveMotor.getConfigurator().apply(driveConfig);

        m_rotationMotor = rotationMotor;
        m_rotationMotor.setNeutralMode(NeutralModeValue.Brake);
        m_rotationMotor.setInverted(true);

        m_encoderOffset = encoderOffset;

        m_rotationPID = new PIDController(
            Constants.SwerveModuleConstants.ROTATION_MOTOR_P,
            Constants.SwerveModuleConstants.ROTATION_MOTOR_I,
            Constants.SwerveModuleConstants.ROTATION_MOTOR_D);
        m_rotationPID.setTolerance(Constants.SwerveModuleConstants.ROTATION_TOLERANCE);
        // tells pid controller that its ok to rotate 20 degrees from 10 to (0 or 360) to 350 rather than going 340 degrees 
        m_rotationPID.enableContinuousInput(0, 360);

        m_cancoder = cancoder;
        MagnetSensorConfigs cancoderConfig = new MagnetSensorConfigs();
        cancoderConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoderConfig.MagnetOffset = 0;
        cancoderConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        m_cancoder.getConfigurator().apply(cancoderConfig);
    }
    
    /** takes in inputState from DriveTrainSubsystem */
    public void setSwerveModuleStates(SwerveModuleState inputState) {
        Rotation2d rotation = new Rotation2d(Math.toRadians(getRotationAngle()));
        SwerveModuleState swerveModuleState = SwerveModuleState.optimize(inputState, rotation);
        setDriveSpeed(swerveModuleState.speedMetersPerSecond);
        setRotationPosition(swerveModuleState.angle.getDegrees());
    }

    /** DriveTrainSubsystem gets these values for odometry purposes */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveMotor.getPosition().getValue() * Constants.SwerveModuleConstants.DIST_PER_MOTOR_ROTATION,
            new Rotation2d(Math.toRadians(getRotationAngle()))
        );
    }
    public void setBrakeMode(boolean brakeMode){
        if(brakeMode) {
            m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
            m_rotationMotor.setNeutralMode(NeutralModeValue.Brake);
        } else {
            m_driveMotor.setNeutralMode(NeutralModeValue.Coast);
            m_rotationMotor.setNeutralMode(NeutralModeValue.Coast);
        }
    }

    public void setDriveSpeed(double inputSpeed) { 
        // because the robot's max speed is 3 m/s, dividing the speed by 3 results in a power [-1, 1] we can set the motor to 
        m_driveMotor.set(inputSpeed / Constants.DriveTrainConstants.ROBOT_MAX_SPEED); // probably should be done outside of swerve module
    }

    public void setRotationPosition(double angle) {
        m_rotationMotor.set(m_rotationPID.calculate(getRotationAngle(), angle));
    }

    /**
     * Determines the encoder offset from the swerve module.
     *<p>
     * Applies this offset to the swerve module.
     * <p>
     * Wheels MUST be pointed to 0 degrees relative to robot to use this method
     * @return new encoder offset
     */
    public double setEncoderOffset(){ 
       double rotationAngle = m_cancoder.getAbsolutePosition().getValue() * 360; 
       m_encoderOffset = rotationAngle;
       return m_encoderOffset;
    }

    public double getRotationAngle() {
        double rotationAngle;
        rotationAngle = m_cancoder.getAbsolutePosition().getValue() * 360 - m_encoderOffset;
        rotationAngle = rotationAngle % 360;
        if(rotationAngle < 0){
            rotationAngle += 360;
        }
        return rotationAngle;        
    }

    public void periodic() {
        // if the DriveTrain subsystem periodic is calling this, this method acts as a periodic
    }
}
