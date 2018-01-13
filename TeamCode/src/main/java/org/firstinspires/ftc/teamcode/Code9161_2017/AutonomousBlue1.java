<<<<<<< HEAD
package org.firstinspires.ftc.teamcode.Code9161_2017;

import android.transition.AutoTransition;

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
import org.firstinspires.ftc.teamcode.AutoTransitioner;

@Autonomous(name="AutonomousBlue1", group="Linear Opmode")
public class AutonomousBlue1 extends LinearOpMode {
    Scorpion r = new Scorpion();

    @Override
    public void runOpMode() {
        AutoTransitioner.transitionOnStop(this,"AutonomousBlue1");
        waitForStart();
        r.driveLeftEncoder(1,6);
        //r.pushJewel("Blue");
        r.driveBackwardEncoder(1,12);
        r.turnCounterwiseEncoder(1,12);
        int a = r.decodePictograph();
        r.driveBackwardEncoder(1,18 + a);
        r.turnClockwiseEncoder(1,12);
        r.driveForwardEncoder(1,36);
        r.setLiftToPosition0();
        r.setTrayToPlace();
    }
}
=======
package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutonomousBlue1", group="Linear OpMode")
public class AutonomousBlue1 extends LinearOpMode {
    Scorpion r = new Scorpion();
    int a =0;
    @Override
    public void runOpMode() {
        waitForStart();
        r.driveLeftEncoder(1,6);
        r.pushJewel("Blue");
        r.driveBackwardEncoder(1,12);
        r.turnCounterwiseEncoder(1,12);
        a = r.decodePictograph();
        r.driveBackwardEncoder(1,18 + a);
        r.turnClockwiseEncoder(1,12);
        r.driveForwardEncoder(1,36);
        r.setLiftToPosition0();
    }
}
>>>>>>> 268fe100df8689f28beb5ef9df1c2dc30592deb1
