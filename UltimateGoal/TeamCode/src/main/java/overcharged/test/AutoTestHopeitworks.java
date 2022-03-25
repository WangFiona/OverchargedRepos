package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@Autonomous(name = "AutoTestHopeitworks", group = "Game")
public class AutoTestHopeitworks extends LinearOpMode {

    DcMotor driveLF;
    DcMotor driveLB;
    DcMotor driveRF;
    DcMotor driveRB;

    @Override
    public void runOpMode() throws InterruptedException {
        driveLF = hardwareMap.dcMotor.get("driveLeftFront");
        driveRF = hardwareMap.dcMotor.get("driveRightFront");
        driveLB = hardwareMap.dcMotor.get("driveLeftBack");
        driveRB = hardwareMap.dcMotor.get("driveRightBack");

        driveLF.setDirection(DcMotor.Direction.REVERSE);
        driveLB.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Satus", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Left Front Postion", driveLF.getCurrentPosition());
            telemetry.addData("Right Front Postion", driveRF.getCurrentPosition());
            telemetry.addData("Left Back Postion", driveLB.getCurrentPosition());
            telemetry.addData("Right Back Postion", driveRB.getCurrentPosition());
            telemetry.update();
        }
        }
    }

