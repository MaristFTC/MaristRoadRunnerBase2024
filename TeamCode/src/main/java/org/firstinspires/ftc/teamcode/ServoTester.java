package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;


@TeleOp(name = "Servo Tester", group = "helper")
public class ServoTester extends LinearOpMode {
    int curServo = 0;
    Servo cur = null;
    double curServoPosition = 0;
    private boolean prevDUP;
    private boolean prevDDOWN;
    List<Servo> servos;




    @Override
    public void runOpMode() throws InterruptedException {




        servos = hardwareMap.getAll(Servo.class);
        for (Servo s:servos){
            telemetry.addLine("Name:"  + s.getDeviceName() + " Port: " + s.getPortNumber()  + " Position: " + round(s.getPosition()));
        }
        telemetry.update();
        waitForStart();




        while(!isStopRequested() && opModeIsActive()){
            //get all the servos from the control hub
            servos = hardwareMap.getAll(Servo.class);
            telemetry.addLine("Use the Dpad Up and Down arrows, to select the servo you want to turn");
            telemetry.addLine("Use the left stick y to turn the selected servo; \nThe respective 'position' of the servo will be displayed via telemetry. \nUse this to help you define servo set positions, or to determine whether  a servo needs to be reprogrammed");

            //if there are no servos
            if(servos.size()==0){
                telemetry.addLine("NO SERVOS FOUND");
            }
            else{//otherwise
                //Print out Info about all the servos
                for (int i = 0; i < servos.size(); i++){
                    Servo s = servos.get(i);
                    telemetry.addLine((i==curServo?"!ENABLED! ":"disabled ")+ "---Name:"  + s.getDeviceName() + "---Port: " + s.getPortNumber() + "---Position: " + s.getPosition());
                }


                //SELECTING SERVO
                if(gamepad1.dpad_up || gamepad1.dpad_down){
                    if(gamepad1.dpad_up && gamepad1.dpad_up !=prevDUP){
                        curServo -=1;
                    }
                    if(gamepad1.dpad_down && gamepad1.dpad_down !=prevDDOWN){
                        curServo+=1;
                    }


                    if(curServo > servos.size()-1 && curServo!=0){curServo=0;}
                    else if(curServo < 0){curServo =servos.size()-1;}


                    if(curServo <servos.size() && curServo >=0 && servos.get(curServo)!=null){
                        cur = servos.get(curServo);
                        curServoPosition = cur.getPosition();
                    }


                }
                prevDUP = gamepad1.dpad_up;
                prevDDOWN = gamepad1.dpad_down;


                //MOVING SERVO
                curServoPosition += gamepad1.left_stick_y*.0005;
                curServoPosition = Range.clip(curServoPosition, 0, 1);
                if(cur != null){cur.setPosition(curServoPosition);}
                telemetry.addLine("TARGET: " + curServoPosition);
                telemetry.update();


            }


        }
    }

    public double round(double input){
        return ((int)(input*100.0))/100.0;
    }











}

