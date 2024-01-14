package org.firstinspires.ftc.teamcode.based.command;

import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.based.subsystem.Drivetrain;

public class DriveCommand extends RunCommand {
    public DriveCommand(Drivetrain drivetrain){
        super(drivetrain::update,
                drivetrain);
    }
}
