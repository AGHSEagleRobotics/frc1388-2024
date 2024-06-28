// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.Constants;

/** Add your docs here. */
public class VisionAcceptor {
    public static final double robotMargin = 0.5;
    
    Twist2d m_robotVelocity;

    public boolean shouldAccept(Pose2d currentPosition, Pose2d lastPosition, Twist2d robotVelocity) {
        
        m_robotVelocity = robotVelocity;

        //first check of position
        if(lastPosition == null || (lastPosition.getX() == 0.0 && lastPosition.getY() == 0.0)) {
            return true;
        }

        //checks if robot is outside of field
        if(currentPosition.getX() < -robotMargin 
          || currentPosition.getX() > Constants.FieldLayout.FIELD_LENGTH + robotMargin
          || currentPosition.getY() < -robotMargin
          || currentPosition.getY() > Constants.FieldLayout.FIELD_WIDTH + robotMargin) 
            return false;
        
        //checks if robot is moving too fast for camera to update
        if(norm() > 4.0) {
            return false;
        }

        // check if the current position compared to the last position is greater than the max velocity of the robot
        if(Math.abs(currentPosition.getX() - lastPosition.getX()) > 0.2
        || Math.abs(currentPosition.getY() - lastPosition.getY()) > 0.2) 
            return false;

        return true;
    }

    public double norm() {
        double dx = m_robotVelocity.dx;
        double dy = m_robotVelocity.dy;
        if (dy == 0.0)
            return Math.abs(dx);
        return Math.hypot(dx, dy);
    }
}
