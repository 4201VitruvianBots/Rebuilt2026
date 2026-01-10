package frc.team4201.lib.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.*;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.function.FloatConsumer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.time.Instant;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.function.LongConsumer;

public class TalonFXTuner implements AutoCloseable {
  private final TalonFX m_talon;
  private final TalonFXConfiguration m_defaultConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_runningConfig;

  private final NetworkTableInstance nt_instance = NetworkTableInstance.getDefault();
  private final NetworkTable nt_subsystem;
  private final NetworkTable nt_motor_instance;

  private final NetworkTableListenerPoller poller = new NetworkTableListenerPoller(nt_instance);
  private final Notifier notifier = new Notifier(this::update);
  private static final EnumSet<Kind> LISTENER_EVENT_KINDS = EnumSet.of(Kind.kValueAll);

  private final Map<String, Map<Integer, ?>> nt_map = new HashMap<>();
  private final Map<Integer, DoubleConsumer> nt_doubleValues = new HashMap<>();
  private final Map<Integer, FloatConsumer> nt_floatValues = new HashMap<>();
  private final Map<Integer, LongConsumer> nt_intValues = new HashMap<>();
  private final Map<Integer, BooleanConsumer> nt_booleanValues = new HashMap<>();
  private final Map<String, BooleanConsumer> nt_booleanStringCallbacks = new HashMap<>();

  private final StringPublisher updateMessagePublisher;

  //    private final int resetFlagHandle;
  //    private final BooleanConsumer resetFlagCallback;

  public TalonFXTuner(TalonFX talon) {
    this(talon, "Unassigned");
  }

  public TalonFXTuner(TalonFX talon, SubsystemBase subsystem) {
    this(talon, subsystem.getName());
  }

  public TalonFXTuner(TalonFX talon, String subsystemName) {
    m_talon = talon;
    m_talon.getConfigurator().refresh(m_defaultConfig);
    m_runningConfig = m_defaultConfig;
    nt_subsystem = nt_instance.getTable(subsystemName);
    nt_motor_instance = nt_subsystem.getSubTable(talon.getDescription());

    addTunableValue(
        "kMaxOutput+",
        m_defaultConfig.MotorOutput.PeakForwardDutyCycle,
        v -> m_runningConfig.MotorOutput.withPeakForwardDutyCycle(v));
    addTunableValue(
        "kMaxOutput-",
        m_defaultConfig.MotorOutput.PeakReverseDutyCycle,
        v -> m_runningConfig.MotorOutput.withPeakReverseDutyCycle(v));
    addTunableValue(
        "NeutralMode",
        m_defaultConfig.MotorOutput.NeutralMode == NeutralModeValue.Brake,
        v ->
            m_runningConfig.MotorOutput.withNeutralMode(
                v ? NeutralModeValue.Brake : NeutralModeValue.Coast),
        new String[] {NeutralModeValue.Coast.name(), NeutralModeValue.Brake.name()});
    addTunableValue("kP", m_defaultConfig.Slot0.kP, v -> m_runningConfig.Slot0.withKP(v));
    addTunableValue("kI", m_defaultConfig.Slot0.kI, v -> m_runningConfig.Slot0.withKI(v));
    addTunableValue("kD", m_defaultConfig.Slot0.kD, v -> m_runningConfig.Slot0.withKD(v));
    addTunableValue("kV", m_defaultConfig.Slot0.kV, v -> m_runningConfig.Slot0.withKV(v));
    addTunableValue("kA", m_defaultConfig.Slot0.kA, v -> m_runningConfig.Slot0.withKA(v));
    addTunableValue("kS", m_defaultConfig.Slot0.kS, v -> m_runningConfig.Slot0.withKS(v));
    addTunableValue("kG", m_defaultConfig.Slot0.kG, v -> m_runningConfig.Slot0.withKG(v));
    addTunableValue(
        "kMMV",
        m_defaultConfig.MotionMagic.MotionMagicCruiseVelocity,
        v -> m_runningConfig.MotionMagic.withMotionMagicCruiseVelocity(v));
    addTunableValue(
        "kMMA",
        m_defaultConfig.MotionMagic.MotionMagicAcceleration,
        v -> m_runningConfig.MotionMagic.withMotionMagicAcceleration(v));

    updateMessagePublisher = nt_motor_instance.getStringTopic("Last Update").publish();

    //        var resetFlagTopic = nt_motor_instance.getBooleanTopic("Reset to Default Configs");
    //        var resetFlagEntry = resetFlagTopic.getEntry(false);
    //        resetFlagEntry.set(false);
    //        resetFlagHandle = poller.addListener(resetFlagEntry, LISTENER_EVENT_KINDS);
    //        resetFlagCallback = v -> {
    //            if(v) {
    //                m_runningConfig = m_defaultConfig;
    //                resetFlagEntry.set(false);
    //            }
    //        };
    //        nt_booleanValues.put(resetFlagHandle, resetFlagCallback);
    //        nt_map.put("Reset to Default Configs", nt_floatValues);

    notifier.startPeriodic(0.05);
  }

  private void addTunableValue(String key, double defaultValue, DoubleConsumer onUpdate) {
    var topic = nt_motor_instance.getDoubleTopic(key);
    var entry = topic.getEntry(defaultValue);

    entry.set(defaultValue);

    var handler = poller.addListener(entry, LISTENER_EVENT_KINDS);

    nt_doubleValues.put(handler, onUpdate);
    nt_map.put(key, nt_doubleValues);
  }

  private void addTunableValue(String key, float defaultValue, FloatConsumer onUpdate) {
    var topic = nt_motor_instance.getFloatTopic(key);
    var entry = topic.getEntry(defaultValue);

    entry.set(defaultValue);

    var handler = poller.addListener(entry, LISTENER_EVENT_KINDS);

    nt_floatValues.put(handler, onUpdate);
    nt_map.put(key, nt_floatValues);
  }

  private void addTunableValue(String key, int defaultValue, LongConsumer onUpdate) {
    var topic = nt_motor_instance.getFloatTopic(key);
    var entry = topic.getEntry(defaultValue);

    entry.set(defaultValue);

    var handler = poller.addListener(entry, LISTENER_EVENT_KINDS);

    nt_intValues.put(handler, onUpdate);
    nt_map.put(key, nt_intValues);
  }

  private void addTunableValue(String key, boolean defaultValue, BooleanConsumer onUpdate) {
    var topic = nt_motor_instance.getBooleanTopic(key);
    var entry = topic.getEntry(defaultValue);

    entry.set(defaultValue);

    var handler = poller.addListener(entry, LISTENER_EVENT_KINDS);

    nt_booleanValues.put(handler, onUpdate);
    nt_map.put(key, nt_booleanValues);
  }

  private void addTunableValue(
      String key, boolean defaultValue, BooleanConsumer onUpdate, String[] values) {
    addTunableValue(key, defaultValue, onUpdate);
    var topic = nt_motor_instance.getStringTopic(key + "_value").publish();
    topic.set(defaultValue ? values[1] : values[0]);

    nt_booleanStringCallbacks.put(key, v -> topic.accept(v ? values[1] : values[0]));
  }

  private void update() {
    var updates = poller.readQueue();
    boolean configSet = false;

    if (updates.length > 0) {
      for (var update : updates) {
        var topicFullString = update.valueData.getTopic().getName();
        var topicName = topicFullString.substring(topicFullString.lastIndexOf("/") + 1);
        var valueMap = nt_map.get(topicName);
        var callback = valueMap.get(update.listener);

        //                if (update.listener == resetFlagHandle) {
        //                    resetFlagCallback.accept(update.valueData.value.getBoolean());
        // TODO: Reset all NT values
        //                } else
        if (callback != null) {
          switch (update.valueData.value.getType()) {
            case kDouble -> ((DoubleConsumer) callback).accept(update.valueData.value.getDouble());
            case kFloat -> ((FloatConsumer) callback).accept(update.valueData.value.getFloat());
            case kInteger -> ((LongConsumer) callback).accept(update.valueData.value.getInteger());
            case kBoolean -> {
              ((BooleanConsumer) callback).accept(update.valueData.value.getBoolean());
              nt_booleanStringCallbacks.get(topicName).accept(update.valueData.value.getBoolean());
            }
              // case kString -> ((Consumer<String>)
              // callback).accept(update.valueData.value.getString());
            default -> {}
          }
          configSet = true;
        }
      }
      if (configSet) {
        var statusCode = m_talon.getConfigurator().apply(m_runningConfig);

        if (statusCode.isOK()) {
          // Set a timestamp string to notify users when an update is applied
          var fullTimestampString = Instant.now().toString();
          updateMessagePublisher.accept(
              "[INFO] Last Update: "
                  + fullTimestampString.substring(0, fullTimestampString.lastIndexOf(".")));
        } else {
          // Set a timestamp string to notify users when an update is applied
          updateMessagePublisher.accept(
              "[WARN] Last Update returned Status Code " + statusCode.getName());
        }
      }
    }
  }

  @Override
  public void close() throws Exception {
    poller.close();
    notifier.close();
  }
}
