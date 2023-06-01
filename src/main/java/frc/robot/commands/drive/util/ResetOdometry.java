// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class ResetOdometry extends InstantCommand {
    public ResetOdometry () {
        addRequirements(RobotContainer.swerveDrive);
    }

    @Override
    public void initialize () {
        RobotContainer.swerveDrive.setCurPose2d(new Pose2d());
    }

    @Override
    public boolean runsWhenDisabled () {
        return true;
    }
}
