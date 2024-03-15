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
    private static SendableChooser<Objective> m_autoObjective = new SendableChooser<>();
    // public final ShooterAngleSubsystem m_shooterAngleSubsystem = new ShooterAngleSubsystem(null, new AnalogPotentiometer(ShooterAngleSubsystemConstants.kPotentiometerAnalogIN));

    final static GenericEntry numWait = (Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME).add("AutoDelay", 15.0).withWidget(BuiltInWidgets.kNumberSlider).getEntry());
    static double numSec = numWait.getDouble(0.0);
    
    public Dashboard() {
        m_shuffleboardTab =  Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME);
        Shuffleboard.selectTab(SHUFFLEBOARD_TAB_NAME);

        HttpCamera limelightIntakeCamera = new HttpCamera("limelight-intake", "http://limelight-intake.local:5800/");
        CameraServer.addCamera(limelightIntakeCamera);
        Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME).add(limelightIntakeCamera)
        .withPosition(0, 0)
        .withSize(20, 14);

        for (int i = 0; i <= 15; i++){
            
        }
        
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
    public static double getWaitTime(){
        return numSec;
    }
}