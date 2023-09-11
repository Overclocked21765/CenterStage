package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

public class DriveCommand extends RunCommand {
    public DriveCommand(DriveSubsystem driveSubsystem){
        super(driveSubsystem::drive, driveSubsystem);
    }
}
