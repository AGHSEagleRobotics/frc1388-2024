package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.MjpegServer;
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
    private final GenericEntry m_canYouShoot;


    
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
        .withSize(6, 8)
        .withPosition(20, 0)
        .getEntry();


    } //end constructor

    public void setIfCanShoot(boolean isReset) {
        m_canYouShoot.setBoolean(isReset);
    }
}