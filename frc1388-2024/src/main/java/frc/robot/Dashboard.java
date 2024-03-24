package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.Objective;


public class Dashboard {
    private final ShuffleboardTab m_shuffleboardTab;
    private final static String SHUFFLEBOARD_TAB_NAME = "Competition";
    
    private final ComplexWidget m_complexWidgetObjective;
    private static SendableChooser<Objective> m_autoObjective = new SendableChooser<>();
    private final ComplexWidget m_complexWidgetDelay;
    private static SendableChooser<Integer> m_autoDelay = new SendableChooser<>();
    
    public Dashboard() {
        m_shuffleboardTab =  Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME);
        Shuffleboard.selectTab(SHUFFLEBOARD_TAB_NAME);

        // Camera view
        HttpCamera limelightIntakeCamera = new HttpCamera("limelight-intake", "http://limelight-intake.local:5800/");
        CameraServer.addCamera(limelightIntakeCamera);
        Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME).add(limelightIntakeCamera)
        .withPosition(0, 0)
        .withSize(20, 14);

        //objectives
        for (AutoConstants.Objective o : Objective.values()) {
            m_autoObjective.addOption(o.getDashboardDescript(), o);
        }
        m_autoObjective.setDefaultOption(Objective.Default.getDashboardDescript(), Objective.Default);
        m_complexWidgetObjective = m_shuffleboardTab.add( "AutoObjective", m_autoObjective)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withSize(8, 4)
        .withPosition(20, 4);

        // Auto Delay options
        m_autoDelay.addOption("0", 0);
        m_autoDelay.addOption("1", 1);
        m_autoDelay.addOption("2", 2);
        m_autoDelay.addOption("3", 3);
        m_autoDelay.addOption("4", 4);
        m_autoDelay.addOption("5", 5);
        m_autoDelay.addOption("6", 6);
        m_autoDelay.addOption("7", 7);
        m_autoDelay.addOption("8", 8);
        m_autoDelay.addOption("9", 9);
        m_autoDelay.addOption("10", 10);
        m_autoDelay.setDefaultOption("0", 0);
        m_complexWidgetDelay = m_shuffleboardTab.add("AutoDelay", m_autoDelay)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(20, 10)
                .withSize(8, 4);
        
    } //end constructor
    
    
    public Objective getObjective() {
        return m_autoObjective.getSelected();
    }
    
    public double getWaitTime(){
        return m_autoDelay.getSelected();
    }
}