package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;

public class CommandBuilder {
    static LinkedList<Subsystem> subsystemList = new LinkedList<>();

    public static void addSubsystems(Subsystem... subsystems) {
        subsystemList.addAll(List.of(subsystems));
    }

    public static void buildCommands() {

    }
}
