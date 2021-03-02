package frc.robot;

import java.util.ArrayList;
import java.util.List;

public class Init {
    private static final List<String> list = new ArrayList<>();

    public static boolean isInit(String key) {
        if (list.contains(key)) {
            return true;
        } else {
            list.add(key);
            return false;
        }
    }
}
