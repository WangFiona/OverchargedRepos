package overcharged.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.robot.Robot;
import overcharged.components.Button;
import overcharged.components.Encoder;
import overcharged.components.Robot6Wheel;

@Config
@TeleOp(name="TeleOp")
public class NewTeleOp extends OpMode {

    Robot6Wheel robot;

    // Intake States
    boolean intakeOn = false;
    boolean intakeOut = false;

    // Flywheel State
    boolean flywheelOn = false;

    // Slide State
    boolean slideUp = false;

    // Tray State
    boolean trayUp = false;

    // Door State
    boolean doorUp = true;

    // Claw State
    boolean clawOpen = false;

    // Wobble State
    boolean wobbleInit = true;
    boolean wobbleUp = false;

    // Auto Wobble State
    boolean isAutoWobbleClosed = true;

    // Automation Variables
    long outtakeTime = System.currentTimeMillis();
    boolean outtakeStart = false;
    boolean step1 = false;
    boolean step2 = false;
    boolean step3 = false;
    boolean step4 = false;
    boolean step5 = false;

    boolean slideDisabled = true;
    boolean wobbleDisabled = true;
    boolean trayDisabled = true;

    boolean wobbleMoved = false;

    double powerMult = 1;
    boolean isSlow = false;

    public static double p = 82.524;
    public static double i = 0.000025;
    public static double d = 9.1;
    public static double f = 19.3;

    public static double pwr = 0.51925;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot6Wheel(this, false);
        this.gamepad1.setJoystickDeadzone(0.1f);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        long timestamp = System.currentTimeMillis();
        robot.drive.setTank(-gamepad1.left_stick_y * powerMult, -gamepad1.right_stick_y * powerMult);

        robot.flywheel.setPIDFCoefficients(p,i,d,f);

        if(gamepad1.right_bumper && Button.BTN_SLOW_MODE.canPress(timestamp)){
            if(!isSlow){
                powerMult = 0.4;
                isSlow = true;
            } else {
                powerMult = 1;
                isSlow = false;
            }
        }

        // Intake Controls
        if(gamepad1.right_trigger > 0.9 && Button.BTN_AUTO.canPress(timestamp)){
            if(!intakeOn){
                robot.intakeOn();
                robot.doorUp();
                doorUp = true;
                intakeOn = true;
            } else {
                robot.intakeOff();
                robot.doorDown();
                doorUp = false;
                intakeOn = false;
            }
            intakeOut = false;
        }
        if(gamepad1.left_trigger > 0.9 && Button.BTN_AUTOMATION.canPress(timestamp)){
            if(!intakeOut){
                robot.intakeOut();
                robot.doorUp();
                doorUp = true;
                intakeOut = true;
            } else {
                robot.intakeOff();
                robot.doorDown();
                doorUp = false;
                intakeOut = false;
            }
            intakeOn = false;
        }

        /*// Flywheel Controls
        if(gamepad2.y && Button.BTN_BACK.canPress(timestamp)){
            if(!flywheelOn) {
                robot.flywheelOn();
                flywheelOn = true;
            } else {
                robot.flywheelOff();
                flywheelOn = false;
            }
        }*/

        if(gamepad1.left_bumper && Button.BTN_BACK.canPress(timestamp)){
            if(!doorUp) {
                robot.doorUp();
                doorUp = true;
            } else {
                if(!trayUp){
                    robot.doorDown();
                    doorUp = false;
                }
            }
        }

        // Automation Code
        if(gamepad2.b && Button.BTN_COLLECT.canPress(timestamp)){
            if(!step1 && !step2){
                outtakeStart = true;
                if(doorUp){
                    robot.doorDown();
                    doorUp = false;
                }
                robot.flywheelMid();
                flywheelOn = true;
                step1 = true;
                robot.intakeOff();
                intakeOn = false;
                intakeOut = false;
                outtakeTime = System.currentTimeMillis();
            } else if(step2) {
                outtakeStart = false;
                robot.flywheelOff();
                flywheelOn = false;
                robot.trayDown();
                trayUp = false;
                robot.doorUp();
                doorUp = false;
                step1 = false;
                step2 = false;
            }
        }
        if(gamepad2.a && Button.BTN_COLLECT.canPress(timestamp)){
            if(!step1 && !step2){
                outtakeStart = true;
                if(doorUp){
                    robot.doorDown();
                    doorUp = false;
                }
                robot.flywheelOn();
                flywheelOn = true;
                step1 = true;
                robot.intakeOff();
                intakeOn = false;
                intakeOut = false;
                outtakeTime = System.currentTimeMillis();
            } else if(step2) {
                outtakeStart = false;
                robot.flywheelOff();
                flywheelOn = false;
                robot.trayDown();
                trayUp = false;
                robot.doorUp();
                doorUp = false;
                step1 = false;
                step2 = false;
            }
        }

        if(outtakeStart && System.currentTimeMillis()-outtakeTime > 300 && step1){
            step1 = false;
            robot.trayUp();
            trayUp = true;
            step2 = true;
            outtakeTime = System.currentTimeMillis();
        }

        if(gamepad2.x && outtakeStart && System.currentTimeMillis()-outtakeTime > 700 && step2) {
            step2 = false;
            robot.doorUp();
            doorUp = true;
            step3 = true;
            outtakeTime = System.currentTimeMillis();
        }
        if(outtakeStart && System.currentTimeMillis()-outtakeTime > 300 && step3){
            step3 = false;
            robot.slideUp();
            slideUp = true;
            step4 = true;
            outtakeTime = System.currentTimeMillis();
        }
        if(outtakeStart && System.currentTimeMillis()-outtakeTime > 500 && step4){
            step4 = false;
            robot.flywheelOff();
            flywheelOn = false;
            robot.slideDown();
            slideUp = false;
            robot.trayDown();
            trayUp = false;
            step5 = true;
        }
        if(outtakeStart && System.currentTimeMillis()-outtakeTime > 1000 && step5){
            outtakeStart = false;
            robot.intakeOn();
            isSlow = false;
            powerMult = 1;
            intakeOn = true;
            intakeOut = false;
            step5 = false;
        }

        /*if(slideDisabled){
            robot.slide.savox.getController().pwmDisable();
        } else {
            robot.slide.savox.getController().pwmEnable();
        }

        if(trayDisabled){
            robot.tray.tray.getController().pwmDisable();
        } else {
            robot.tray.tray.getController().pwmEnable();
        }

        if(wobbleDisabled){
            robot.wobble.arm.getController().pwmDisable();
        } else {
            robot.wobble.arm.getController().pwmEnable();
        }*/

        if(gamepad2.y && Button.BTN_WOBBLE_ARM.canPress(timestamp)){
            if(!wobbleMoved){
                robot.wobble.enable();
                wobbleMoved = true;
            }
            if(wobbleInit){
                robot.wobbleDown();
                wobbleUp = false;
                wobbleInit = false;
                robot.wobbleOpen();
                clawOpen = true;
            } else if(!wobbleUp && !wobbleInit){
                robot.wobbleUp();
                wobbleUp = true;
                wobbleInit = false;
            } else if(wobbleUp) {
                robot.wobbleInit();
                wobbleInit = true;
                robot.wobbleClose();
                clawOpen = false;
            }
        }


        if(gamepad1.y && Button.BTN_WOBBLE_CLAW.canPress(timestamp)){
            if(!clawOpen && !wobbleInit){
                robot.wobbleOpen();
                clawOpen = true;
            } else {
                robot.wobbleClose();
                clawOpen = false;
            }
        }

        if(gamepad1.b && Button.BTN_DISABLE.canPress(timestamp)){
            if(isAutoWobbleClosed){
                robot.autoWobbleUp();
                isAutoWobbleClosed = false;
            } else {
                robot.autoWobbleDown();
                isAutoWobbleClosed = true;
            }
        }

        /*double pos = robot.tray.door.getPosition();
        if(gamepad2.left_bumper){
            pos -= 0.01;
        }
        if(gamepad2.left_trigger > 0.9){
            pos += 0.01;
        }
        robot.tray.setDoorPosition(pos);*/

        telemetry.addData("Door Open", doorUp);
        telemetry.addData("Slow Mode", isSlow);
        telemetry.addData("Flywheel Velo", Math.abs(robot.flywheel.left.getVelocity()));
        telemetry.update();
    }
}
