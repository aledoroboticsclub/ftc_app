package org.firstinspires.ftc.teamcode.Code2018.Interview;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="InterviewColorSensor", group="Iterative Opmode")
//@Disabled
public class InterviewColorSensor extends OpMode
{
    ColorSensor color=null;

    @Override
    public void init() {
        color=hardwareMap.get(ColorSensor.class, "color");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        telemetry.addData("Blue: ", color.blue());
        telemetry.addData("Red: ", color.red());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

}
