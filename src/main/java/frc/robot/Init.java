package frc.robot;

import java.util.ArrayList;
import java.util.List;

public class Init {
    private final List<String> list = new ArrayList<>();

    public boolean isNotInit(String key) {
        if (list.contains(key)) {
            return false;
        } else {
            list.add(key);
            return true;
        }
    }

    public void resetInit(String key) {
        list.remove(key);
    }

    public void resetAll() {
        list.clear();
    }
}
