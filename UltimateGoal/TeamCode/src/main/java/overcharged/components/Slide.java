package overcharged.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Slide {

    public Servo savox;

    public static double down = 0.94;
    public static double pos1 = 0.52;
    public static double pos2 = 0.34;
    public static double up = 0.21;

    public Slide(HardwareMap hardwareMap){
        savox = hardwareMap.servo.get("slide");
        savox.setDirection(Servo.Direction.FORWARD);
        savox.setPosition(down);
    }

    public void setPosition(double pos){
        savox.setPosition(pos);
    }

    public void disable(){
        savox.getController().pwmDisable();
    }

    public void enable() {
        savox.getController().pwmEnable();
    }
}
