package overcharged.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Flywheel1 {

    public DcMotorEx left;

    public VoltageSensor battery;

    public Flywheel1(HardwareMap hardwareMap){
        left = hardwareMap.get(DcMotorEx.class, "flywheelL");

        left.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(82.524,0.000025,9.1,19.5));

        battery = hardwareMap.voltageSensor.iterator().next();
    }

    /*public void on(){
        left.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(90,0,55,7));
        right.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(90,0,55,7));
        double voltage = battery.getVoltage() < 12.7 ? (8.6/battery.getVoltage()) * (battery.getVoltage()/15.4) : 8.51/battery.getVoltage();
        if(voltage < 0.62) voltage = 0.62;
        else if(voltage > 0.71) voltage = 0.71;
        left.setPower(voltage);
        right.setPower(voltage);
    }*/

    public void on(){
        left.setPower(0.5);
    }

    public void mid(){
        left.setPower(0.44);
    }

    public void on(double pwr){
        left.setPower(pwr);
    }

    public void max(){
        left.setPower(1);
    }

    public void off(){
        left.setPower(0);
    }

    public double getVelocity(){
        return (Math.abs(left.getVelocity()));
    }

    public void setPIDFCoefficients(double p, double i, double d, double f){
        left.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p,i,d,f));
    }

    public void setPIDFCoefficients(PIDFCoefficients pid){
        left.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
    }

}
