package frc.robot.util;

import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NetworkTablesAgent extends SubsystemBase {
    private final StringSubscriber buttonSub;
    public final AtomicReference<String> buttonValue = new AtomicReference<String>("N");

    private final DoubleSubscriber triggerSub;
    public final AtomicReference<Double> triggerValue = new AtomicReference<Double>(0.0);

    private final StringSubscriber upDownSub;
    public final AtomicReference<String> upDownValue = new AtomicReference<String>("N");

    private final StringSubscriber elevatorClimbSub;
    public final AtomicReference<String> elevatorClimbSwitchValue = new AtomicReference<String>("N");

    public NetworkTablesAgent() {
         NetworkTableInstance inst = NetworkTableInstance.getDefault();
         NetworkTable datatable = inst.getTable("Arduino");
         buttonSub = datatable.getStringTopic("Reef").subscribe("N");
         triggerSub = datatable.getDoubleTopic("Level").subscribe(0);
         upDownSub = datatable.getStringTopic("Climb").subscribe("N");
         elevatorClimbSub = datatable.getStringTopic("ClimbSwitch").subscribe("C");

        inst.addListener(buttonSub, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            buttonValue.set(event.valueData.value.getString());
        });
        inst.addListener(triggerSub, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            triggerValue.set(event.valueData.value.getDouble());
        });
        inst.addListener(upDownSub, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            upDownValue.set(event.valueData.value.getString());
        });
        inst.addListener(elevatorClimbSub, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            elevatorClimbSwitchValue.set(event.valueData.value.getString());
        });
    }
}
