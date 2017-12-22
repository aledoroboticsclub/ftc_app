package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class StriderAutonomous extends LinearOpMode {
    Strider s = new Strider();
    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
