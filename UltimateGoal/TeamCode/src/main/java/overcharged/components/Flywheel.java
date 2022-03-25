package overcharged.components;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Flywheel {

    public DcMotorEx left;
    public DcMotorEx right;

    public VoltageSensor battery;

    public Flywheel(HardwareMap hardwareMap){
        left = hardwareMap.get(DcMotorEx.class, "flywheelL");
        right = hardwareMap.get(DcMotorEx.class, "flywheelR");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(82.524,0.000025,9.1,19.5));
        right.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(82.524,0.000025,9.1,19.5));

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
        left.setPower(0.485);
        right.setPower(0.485);
    }

    public void mid(){
        left.setPower(0.44);
        right.setPower(0.44);
    }

    public void on(double pwr){
        left.setPower(pwr);
        right.setPower(pwr);
    }

    public void max(){
        left.setPower(1);
        right.setPower(1);
    }

    public void off(){
        left.setPower(0);
        right.setPower(0);
    }

    public double getVelocity(){
        return (Math.abs(left.getVelocity()) + Math.abs(right.getVelocity()))/2;
    }

    public void setPIDFCoefficients(double p, double i, double d, double f){
        left.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p,i,d,f));
        right.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p,i,d,f));
    }

    public void setPIDFCoefficients(PIDFCoefficients pid){
        left.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
        right.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
    }
}
