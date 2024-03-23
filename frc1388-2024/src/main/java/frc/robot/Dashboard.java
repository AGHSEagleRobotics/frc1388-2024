package frc.robot;
import java.util.Map;
import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.Objective;
import frc.robot.Constants.ShooterAngleSubsystemConstants;
import frc.robot.subsystems.ShooterAngleSubsystem;


public class Dashboard {
    private final ShuffleboardTab m_shuffleboardTab;
    private final static String SHUFFLEBOARD_TAB_NAME = "Competition";
    // private final GenericEntry m_canYouShoot;
    // private final SimpleWidget m_shooterPosition;
    private final ComplexWidget m_complexWidgetObjective;
    // private final ComplexWidget m_complexWidgetDelay;
    private static SendableChooser<Objective> m_autoObjective = new SendableChooser<>();
    // private static SendableChooser<Objective> m_autoDelay = new SendableChooser<>();
    // public final ShooterAngleSubsystem m_shooterAngleSubsystem = new ShooterAngleSubsystem(null, new AnalogPotentiometer(ShooterAngleSubsystemConstants.kPotentiometerAnalogIN));



    static GenericEntry numWait = (Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME)
     .add("AutoDelay", 0.0)
     .withWidget(BuiltInWidgets.kNumberSlider)
     .withProperties(Map.of("min", 0, "max", 10))
     .withPosition(20, 10)
     .withSize(8, 4)
     .getEntry());    
    
    public Dashboard() {
        m_shuffleboardTab =  Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME);
        Shuffleboard.selectTab(SHUFFLEBOARD_TAB_NAME);

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
        
    } //end constructor
    
    
    public Objective getObjective() {
        return m_autoObjective.getSelected();
    }
    
    public double getWaitTime(){
        double numSec = numWait.getDouble(0);
        SmartDashboard.putNumber("numberOfSecTestSee", numSec);
        return numWait.getDouble(0);
    }
}