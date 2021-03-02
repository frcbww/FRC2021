package frc.robot;

import edu.wpi.first.wpilibj.*;


public class Print {
    private final Timer print_timer = new Timer();
    String message;

    public Print(){
        print_timer.start();
    }

    public void print (String input){
        if (print_timer.get() > 0.1){
            System.out.println(input);
            print_timer.reset();
        }
    }

    public void print (double input){
        if (print_timer.get() > 0.1){
            System.out.println(input);
            print_timer.reset();
        }
    }

    public void print (boolean input){
        if (print_timer.get() > 0.1){
            System.out.println(input);
            print_timer.reset();
        }
    }

}
