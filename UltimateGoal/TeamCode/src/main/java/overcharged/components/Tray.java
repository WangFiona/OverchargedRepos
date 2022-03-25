package overcharged.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Tray {

    public Servo tray;
    public Servo door;

    public static double down = 0.739;
    public static double up = 0.49;
    public static double doorup = 0.6;
    public static double doordown = 0.83;

    public Tray(HardwareMap hardwareMap){
        tray = hardwareMap.servo.get("tray");
        door = hardwareMap.servo.get("door");

        tray.setDirection(Servo.Direction.FORWARD);
        door.setDirection(Servo.Direction.FORWARD);

        tray.setPosition(down);
        door.setPosition(doorup);
    }

    public void setTrayPosition(double pos){
        tray.setPosition(pos);
    }

    public void setDoorPosition(double pos){
        door.setPosition(pos);
    }

    public void disable() {
        tray.getController().pwmDisable();
    }

    public void enable() {
        tray.getController().pwmEnable();
    }

}
