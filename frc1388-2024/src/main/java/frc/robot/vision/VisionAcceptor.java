// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.random.RandomGenerator.JumpableGenerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class VisionAcceptor {
    public static final double robotMargin = 0.5;
    
    Twist2d m_robotVelocity;
    Pose2d m_lastPosition;
    int m_jumpCount = 0;
    int m_jumpCountMax = 0;

    public boolean shouldAccept(Pose2d currentPosition, Twist2d robotVelocity) {
         m_robotVelocity = robotVelocity;

        if(robotVelocity == null || currentPosition == null) {
            return false;
        }

        // if this is the first ever check, then initialize class variable and trust the position
        if(m_lastPosition == null) {
            m_lastPosition = currentPosition;
            return true;
        }

        if(currentPosition.getX() == 0.0 && currentPosition.getY() == 0.0) {
            return false;
        }

        SmartDashboard.putNumber("difference of x", Math.abs(currentPosition.getX() - m_lastPosition.getX()));
        SmartDashboard.putNumber("difference of y", Math.abs(currentPosition.getY() - m_lastPosition.getY()));

        double velocityPerTick =  norm() / 50;

        double velocityPerTickClamped = MathUtil.clamp(velocityPerTick, 0.03, velocityPerTick);

        SmartDashboard.putNumber("normalizedVelocity", norm());
        SmartDashboard.putNumber("velocityPerTick", velocityPerTick);
        SmartDashboard.putNumber("velocityPerTickClamped", velocityPerTickClamped);

        // check if the current position compared to the last position is greater than the velocity per tick of the robot
        if(Math.abs(currentPosition.getX() - m_lastPosition.getX()) > velocityPerTickClamped
        || Math.abs(currentPosition.getY() - m_lastPosition.getY()) > velocityPerTickClamped) {
            m_jumpCount++;
            if(m_jumpCount > m_jumpCountMax) {
                m_jumpCountMax = m_jumpCount;
            }
            System.out.println("robot position jumped count = " + m_jumpCount + " max = " + m_jumpCountMax);
            m_lastPosition = currentPosition;
            return false;
        }
        else {
            m_jumpCount = 0;
        }

        //checks if robot is outside of field
        if(currentPosition.getX() < -robotMargin 
          || currentPosition.getX() > Constants.FieldLayout.FIELD_LENGTH + robotMargin
          || currentPosition.getY() < -robotMargin
          || currentPosition.getY() > Constants.FieldLayout.FIELD_WIDTH + robotMargin) {
            m_lastPosition = currentPosition;
            return false;
        }

        // checks if robot is moving too fast for camera to update
        if (norm() > 4.0) {
            m_lastPosition = currentPosition;
            return false;
        }

        if (m_robotVelocity.dtheta > 1.2) {
            m_lastPosition = currentPosition;
            return false;
        }

        m_lastPosition = currentPosition;
        
        return true;      
    }

    public boolean shouldResetGyro(Twist2d robotVelocity) {
        if(norm() == 0.0) {
        return true;
        }
    return false;
    }

    public double norm() {
        double dx = m_robotVelocity.dx;
        double dy = m_robotVelocity.dy;
        if (dy == 0.0)
            return Math.abs(dx);
        return Math.hypot(dx, dy);
    }
}
