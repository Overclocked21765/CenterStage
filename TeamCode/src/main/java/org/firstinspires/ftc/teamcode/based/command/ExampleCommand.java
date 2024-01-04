package org.firstinspires.ftc.teamcode.based.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.based.subsystem.ExampleSubsystem;

/**
 * Represents an example of a user defined command
 *
 * Most commands will be declared using llambda notation
 * Thus this type of declaration - creating a custom command class - is typically avoided if possible
 */
public class ExampleCommand extends CommandBase {
    private final ExampleSubsystem m_subsystem; //Subsystems of commands always declared private final, and with m_ in front

    public ExampleCommand(ExampleSubsystem subsystem){
        this.m_subsystem = subsystem;
        this.addRequirements(subsystem);
    }

    //Called on command creation
    @Override
    public void initialize(){

    }

    //Called repeatedly until end
    @Override
    public void execute(){

    }

    //Called when command ends
    @Override
    public void end(boolean interrupted){

    }

    //Determines when command ends
    @Override
    public boolean isFinished(){
        return true;
    }

}
