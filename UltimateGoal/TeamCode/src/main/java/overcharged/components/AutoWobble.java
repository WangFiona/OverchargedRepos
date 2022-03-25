package overcharged.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoWobble {

    public Servo arm;

    public static double up = 0.366;
    public static double down = 0;

    public AutoWobble(HardwareMap hardwareMap){
        arm = hardwareMap.servo.get("autowobble");
        arm.setDirection(Servo.Direction.FORWARD);
        arm.setPosition(down);
    }

    public void setArmDown(){
        arm.setPosition(down);
    }

    public void setArmUp(){
        arm.setPosition(up);
    }

}
