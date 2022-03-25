package overcharged.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    DcMotor left;
    DcMotor right;

    public Intake(HardwareMap hardwareMap){
        left = hardwareMap.dcMotor.get("intakeL");
        right = hardwareMap.dcMotor.get("intakeR");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void on(){
        left.setPower(1);
        right.setPower(1);
    }

    public void off(){
        left.setPower(0);
        right.setPower(0);
    }

    public void out(){
        left.setPower(-1);
        right.setPower(-1);
    }
}
