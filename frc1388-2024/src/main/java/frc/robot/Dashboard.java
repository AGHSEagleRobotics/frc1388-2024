package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveTrainConstants;


public class Dashboard {
    private final ShuffleboardTab m_shuffleboardTab;
    private final String SHUFFLEBOARD_TAB_NAME = "Competition";

    private final UsbCamera m_cameraView;
    private final int CAMERA_RES_WIDTH = 320;
    private final int CAMERA_RES_HEIGHT = 200;
    private final int CAMERA_FPS = 30;


    
    public Dashboard() {
        m_shuffleboardTab =  Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME);
        Shuffleboard.selectTab(SHUFFLEBOARD_TAB_NAME);

        m_cameraView = CameraServer.startAutomaticCapture(0); 
        m_cameraView.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        m_cameraView.setFPS(CAMERA_FPS);
        m_cameraView.setResolution(CAMERA_RES_WIDTH, CAMERA_RES_HEIGHT);

        HttpCamera limelightCamera = new HttpCamera("Limelight", "limelight.local:5801");
        CameraServer.addCamera(limelightCamera);
        Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME).add(limelightCamera);


    } //end constructor

}