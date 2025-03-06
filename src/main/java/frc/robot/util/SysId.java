package frc.robot.util;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysId {
    public static SysIdRoutine getRoutine(
        double rampRate,
        double stepVoltage,
        double seconds,
        String subsystemName,
        Consumer<Double> voltageSetter,
        Supplier<Double> positionSupplier,
        Supplier<Double> velocitySupplier,
        Supplier<Double> voltageSupplier,
        Subsystem subsystemInstance
    ) {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(rampRate).per(Second), 
                Units.Volts.of(stepVoltage), 
                Seconds.of(seconds),
                state -> SignalLogger.writeString(subsystemName + " SysId-State", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> voltageSetter.accept(output.in(Volts)),
                state -> {
                    SignalLogger.writeDouble(subsystemName + " Position", positionSupplier.get());
                    SignalLogger.writeDouble(subsystemName + " Velocity", velocitySupplier.get());
                    SignalLogger.writeDouble(subsystemName + " Voltage", voltageSupplier.get());
                }, 
                subsystemInstance));
    }
}