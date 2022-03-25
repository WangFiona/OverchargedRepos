package overcharged.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble {

    Servo arm;
    Servo claw;

    public static double init = 0.958;
    public static double up = 0.512;
    public static double down = 0.16;
    public static double open = 0.09;
    public static double closed = 0.41;

    public Wobble(HardwareMap hardwareMap){
        arm = hardwareMap.servo.get("arm");
        claw = hardwareMap.servo.get("claw");

        arm.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);

        arm.setPosition(init);
        claw.setPosition(closed);
    }

    public void setArmPosition(double pos){
        arm.setPosition(pos);
    }

    public void setClawPosition(double pos){
        claw.setPosition(pos);
    }

    public void disable() {
        arm.getController().pwmDisable();
    }

    public void enable() {
        arm.getController().pwmEnable();
    }

}
