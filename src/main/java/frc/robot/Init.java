package frc.robot;

import java.util.ArrayList;
import java.util.List;

public class Init {
    private final List<String> list = new ArrayList<>();

    public boolean isNotInit(String key) {
        if (list.contains(key)) {
            return false;
        } else {
            return true;
        }
    }

    public void done(String key){
        if(!list.contains(key)){
            list.add(key);
        }
    }

    public void reset(String key) {
        list.remove(key);
    }

    public void resetAll() {
        list.clear();
    }
}
