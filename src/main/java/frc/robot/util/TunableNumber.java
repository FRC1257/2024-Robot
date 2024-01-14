package frc.robot.util;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class TunableNumber {
    private String key;
    private double defaultValue;
    private double value;

    private LoggedDashboardNumber loggedDashboardNumber;

    public TunableNumber(String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.value = defaultValue;

        loggedDashboardNumber = new LoggedDashboardNumber(key, defaultValue);
    }

    public TunableNumber(String key) {
        this(key, 0.0);
    }

    public TunableNumber(String subsystem, String key, double defaultValue) {
        this.key = "/" + subsystem + "/" + key;
        this.defaultValue = defaultValue;
        this.value = defaultValue;

        loggedDashboardNumber = new LoggedDashboardNumber(this.key, defaultValue);
    }

    public double get() {
        return loggedDashboardNumber.get();
    }

    public void set(double value) {
        this.value = value;
        
        loggedDashboardNumber.set(value);;
    }

    public void reset() {
        set(defaultValue);
    }

    public boolean checkUpdate() {
        double newValue = get();
        if (newValue != value) {
            value = newValue;
            return true;
        }
        return false;
    }

    public void updateFunction(Runnable function) {
        if (checkUpdate()) {
            function.run();
        }
    }
}
