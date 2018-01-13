package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@Autonomous(name="AutonomousRed2", group="Linear OpMode")
public class AutonomousRed2 extends LinearOpMode {
    Scorpion r = new Scorpion();
    int a = 0;
    @Override
    public void runOpMode() {
        waitForStart();
        r.driveLeftEncoder(1,6);
        r.pushJewel("Red");
        r.driveBackwardEncoder(1,12);
        r.turnCounterwiseEncoder(1,12);
        a = r.decodePictograph();
        r.driveLeftEncoder(1,12 + a);
        r.setLiftToPosition0();
    }
}