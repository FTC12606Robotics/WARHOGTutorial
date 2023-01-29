package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import static java.lang.Math.*;

@TeleOp(name="WARHOGDemo", group="")
public class WARHOGDemo extends LinearOpMode{

    double armposChange, intakeArmSpeed = .03, armpos;

    boolean toggleIntakeClaw, toggleOuttakeClaw;

    public WARHOGDemo() throws InterruptedException{};

    @Override
    public void runOpMode() throws InterruptedException{
        Outtake outtake = new Outtake(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        armpos = intake.runArm(Intake.Height.UPRIGHT);

        waitForStart();

        outtake.setTarget(300);
        while(opModeIsActive()){//set up inputs
            try {
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);

                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
            }
            catch (RobotCoreException e) {
                // Swallow the possible exception, it should not happen as
                // currentGamepad1/2 are being copied from valid Gamepads.
            }

            toggleIntakeClaw = (currentGamepad1.a&&!previousGamepad1.a || currentGamepad1.b&&!previousGamepad1.b || currentGamepad1.y&&!previousGamepad1.y || currentGamepad1.x&&!previousGamepad1.x || currentGamepad1.right_bumper&&!previousGamepad1.right_bumper || currentGamepad1.left_bumper&&!previousGamepad1.left_bumper);
            toggleOuttakeClaw = (currentGamepad2.a&&!previousGamepad2.a || currentGamepad2.b&&!previousGamepad2.b || currentGamepad2.y&&!previousGamepad2.y || currentGamepad2.x&&!previousGamepad2.x || currentGamepad2.right_bumper&&!previousGamepad2.right_bumper || currentGamepad2.left_bumper&&!previousGamepad2.left_bumper);

            armposChange = currentGamepad1.left_stick_y*intakeArmSpeed;
            if(abs(currentGamepad1.right_stick_y*intakeArmSpeed)>abs(armposChange)){
                armposChange = currentGamepad2.right_stick_y*intakeArmSpeed;
            }
            armpos += armposChange;

            if(abs(currentGamepad2.left_stick_y)>abs(currentGamepad2.right_stick_y)){
                outtake.run(-currentGamepad2.left_stick_y);
            }
            else{
                outtake.run(-currentGamepad2.right_stick_y);
            }



            telemetry.addData("outside slide value", outtake.showSlideValue());
            telemetry.update();
        }
    }
}
