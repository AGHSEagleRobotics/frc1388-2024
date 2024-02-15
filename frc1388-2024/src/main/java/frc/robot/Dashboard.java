package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.Objective;
import frc.robot.Constants.AutoConstants.Position;


public class Dashboard {
    private final ShuffleboardTab m_shuffleboardTab;
    private final String SHUFFLEBOARD_TAB_NAME = "Competition";
    private final GenericEntry m_canYouShoot;
    private final ComplexWidget m_complexWidgetObjective;
    private final ComplexWidget m_complexWidgetPosition;
    private static SendableChooser<Objective> m_autoObjective = new SendableChooser<>();
    private static SendableChooser<Position> m_autoPosition = new SendableChooser<>();
    
    public Dashboard() {
        m_shuffleboardTab =  Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME);
        Shuffleboard.selectTab(SHUFFLEBOARD_TAB_NAME);

        HttpCamera limelightCamera = new HttpCamera("limelight", "http://limelight.local:5800");
        CameraServer.addCamera(limelightCamera);
        Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME).add(limelightCamera)
        .withPosition(0, 0)
        .withSize(20, 14);

        m_canYouShoot = m_shuffleboardTab.add("Can You Shoot?", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(4, 4)
        .withPosition(20, 0)
        .getEntry();

        //objectives
        for (AutoConstants.Objective o : Objective.values()) {
            m_autoObjective.addOption(o.getDashboardDescript(), o);
        }

        m_autoObjective.setDefaultOption(Objective.Default.getDashboardDescript(), Objective.Default);
        m_complexWidgetObjective = m_shuffleboardTab.add( "AutoObjective", m_autoObjective)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(4, 4)
            .withPosition(20, 4);


        //position
        for (AutoConstants.Position p : Position.values()) {
            m_autoPosition.addOption(p.getDashboardDescript(), p);
        }

        m_autoPosition.setDefaultOption(Position.Default.getDashboardDescript(), Position.Default);               
        m_complexWidgetPosition = m_shuffleboardTab.add( "AutoPosition", m_autoPosition)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(4, 4)
            .withPosition(20, 8);

    } //end constructor

    public void setIfCanShoot(boolean isReset) {
        m_canYouShoot.setBoolean(isReset);
    }

    public Objective getObjective() {
        return m_autoObjective.getSelected();
    }
    public Position getPosition() {
        return m_autoPosition.getSelected();
    }
}