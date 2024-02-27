package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.Objective;
import frc.robot.Constants.ShooterAngleSubsystemConstants;
import frc.robot.subsystems.ShooterAngleSubsystem;


public class Dashboard {
    private final ShuffleboardTab m_shuffleboardTab;
    private final String SHUFFLEBOARD_TAB_NAME = "Competition";
    private final GenericEntry m_canYouShoot;
    // private final SimpleWidget m_shooterPosition;
    private final ComplexWidget m_complexWidgetObjective;
    private static SendableChooser<Objective> m_autoObjective = new SendableChooser<>();
    // public final ShooterAngleSubsystem m_shooterAngleSubsystem = new ShooterAngleSubsystem(null, new AnalogPotentiometer(ShooterAngleSubsystemConstants.kPotentiometerAnalogIN));
    
    public Dashboard() {
        m_shuffleboardTab =  Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME);
        Shuffleboard.selectTab(SHUFFLEBOARD_TAB_NAME);

        HttpCamera limelightShooterCamera = new HttpCamera("limelight", "http://limelight-shooter.local:5800");
        CameraServer.addCamera(limelightShooterCamera);
        Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME).add(limelightShooterCamera)
        .withPosition(0, 0)
        .withSize(20, 14);

        m_canYouShoot = m_shuffleboardTab.add("Can You Shoot?", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(8, 4)
        .withPosition(20, 0)
        .getEntry();

        // m_shooterPosition = m_shuffleboardTab.add("Shooter Position", m_shooterAngleSubsystem.getCurrentPosition())
        //     .withWidget(BuiltInWidgets.kNumberBar)
        //     .withSize(4, 4)
        //     .withPosition(5, 2);

        //objectives
        for (AutoConstants.Objective o : Objective.values()) {
            m_autoObjective.addOption(o.getDashboardDescript(), o);
        }

        m_autoObjective.setDefaultOption(Objective.Default.getDashboardDescript(), Objective.Default);
        m_complexWidgetObjective = m_shuffleboardTab.add( "AutoObjective", m_autoObjective)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(8, 4)
            .withPosition(20, 4);

    } //end constructor

    public void setIfCanShoot(boolean isReset) {
        m_canYouShoot.setBoolean(isReset);
    }

    public Objective getObjective() {
        return m_autoObjective.getSelected();
    }
}