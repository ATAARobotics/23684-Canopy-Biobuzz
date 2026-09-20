package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ArtificialPotentialFieldDrive extends Node {

    private final SafeDevice<DcMotorEx> fl;
    private final SafeDevice<DcMotorEx> fr;
    private final SafeDevice<DcMotorEx> bl;
    private final SafeDevice<DcMotorEx> br;

    RectangularFieldBody hiveOne;
    RectangularFieldBody hiveTwo;
    RectangularFieldBody flowerOne;
    RectangularFieldBody flowerTwo;
    RectangularFieldBody flowerThree;
    RectangularFieldBody flowerFour;


    public class RectangularFieldBody {

        double x, y;
        double fieldX, fieldY;
        double weight;

        List<FieldBody> rectangle;
        List<Double> forces;

        public RectangularFieldBody(double x, double y, double fieldX, double fieldY, double weight) {
            this.x = x;
            this.y = y;
            this.fieldX = fieldX;
            this.fieldY = fieldY;
            this.weight = weight;

            rectangle = new ArrayList<>();
            forces = new ArrayList<>();

            int samplesX = Math.max(1, (int) Math.ceil(x));
            int samplesY = Math.max(1, (int) Math.ceil(y));
            double minX = fieldX - x / 2.0;
            double maxX = fieldX + x / 2.0;
            double minY = fieldY - y / 2.0;
            double maxY = fieldY + y / 2.0;

            // Bottom edge
            for (int i = 0; i <= samplesX; i++) {
                double px = minX + (x * i / samplesX);
                double py = minY;

                rectangle.add(new FieldBody(x, weight, px, py));
            }

            // Top edge
            for (int i = 0; i <= samplesX; i++) {
                double px = minX + (x * i / samplesX);
                double py = maxY;

                rectangle.add(new FieldBody(x, weight, px, py));
            }

            // Left edge
            for (int i = 1; i < samplesY; i++) {
                double px = minX;
                double py = minY + (y * i / samplesY);

                rectangle.add(new FieldBody(y, weight, px, py));
            }

            // Right edge
            for (int i = 1; i < samplesY; i++) {
                double px = maxX;
                double py = minY + (y * i / samplesY);

                rectangle.add(new FieldBody(y, weight, px, py));
            }
        }

        public double[] RepulsiveForceVector(double robotx, double roboty) {

            double shortDis = Double.POSITIVE_INFINITY; //why not
            FieldBody closeFeildBody = null;

            forces.clear();

            for (FieldBody fieldBody : rectangle) {
                double dis = fieldBody.DistanceFromDia(robotx,roboty);

                if(dis <= shortDis ){
                    shortDis = dis;
                    closeFeildBody = fieldBody;
                }
            }

            if (closeFeildBody == null) {
                return new double[]{0,0};
            }

            return closeFeildBody.repulsiveForceVector(robotx,roboty) ;
        }

        public double[] RobotCentricRepulsiveForceVector(double robotx, double roboty,double heading){

            double[] force = RepulsiveForceVector(robotx,roboty);
            double robotForceX = force[0] * Math.cos(heading) + force[1] * Math.sin(heading);

            double robotForceY = -force[0] * Math.sin(heading) + force[1] * Math.cos(heading);

            return new double[]{robotForceX,robotForceY};
        }
    }
    private class FieldBody{
        double dia;
        double fieldX, fieldY;
        double weight;
        public FieldBody(double dia, double weight, double fieldX, double fieldY){
            this.fieldX = fieldX;
            this.fieldY = fieldY;
            this.dia = dia;
            this.weight = weight;
        }

       public double DistanceFromDia(double robotx,double roboty){ //TODO: Make this be safepedro Pose
            double deltax = fieldX - robotx;
            double deltay = fieldY - roboty;

            return Math.sqrt((deltax*deltax) + (deltay*deltay));
        }

        public double[] repulsiveForceVector(double robotx,double roboty){ //TODO: Make this be safepedro Pose
            double deltax = robotx - fieldX;
            double deltay = roboty - fieldY;

            double maguitude;
            if((1.0/ DistanceFromDia(robotx,roboty)-1.0/15) > 0.1) maguitude = (1.0/ DistanceFromDia(robotx,roboty)-1.0/15); //we dont want a math error. that would be quite "skibity" as the youth say
            else maguitude = 0;

            double fx = maguitude * (deltax / DistanceFromDia(robotx,roboty));
            double fy = maguitude * (deltay / DistanceFromDia(robotx,roboty));

            return  new double[]{fx,fy};

        }

//        public double repulsiveForce(){
//            double fin;
//
//            if((0.5*(DistanceFromDia(5,5))-2.5) < 0){
//                fin = 0.5*(DistanceFromDia(5,5))-2.5;  Make this be safepedro Pose
//            }else{
//                fin = 0;
//            }
//
//            return fin;
//        }

    }

    public ArtificialPotentialFieldDrive(Orchestrator orch, SafeDevice<DcMotorEx> frontLeft, SafeDevice<DcMotorEx> frontRight, SafeDevice<DcMotorEx> backLeft, SafeDevice<DcMotorEx> backRight) {
        super(orch);
        this.fl = frontLeft;
        this.fr = frontRight;
        this.bl = backLeft;
        this.br = backRight;

        fl.run(m -> m.setDirection(DcMotor.Direction.FORWARD));
        bl.run(m -> m.setDirection(DcMotor.Direction.FORWARD));
        fr.run(m -> m.setDirection(DcMotor.Direction.REVERSE));
        br.run(m -> m.setDirection(DcMotor.Direction.REVERSE));


    }

    @RunPeriodically(hz = 50, hardware = true)
    public void drive() {
        double y = -orchestrator.getLatestValue("g1/left_stick_y", Float.class).map(Float::doubleValue).orElse(0.0);
        double x = orchestrator.getLatestValue("g1/left_stick_x", Float.class).map(Float::doubleValue).orElse(0.0) * 1.1;
        double r = -orchestrator.getLatestValue("g1/right_stick_x", Float.class).map(Float::doubleValue).orElse(0.0);

        double pFL = y + x + r;
        double pFR = y - x - r;
        double pBL = y - x + r;
        double pBR = y + x - r;
        double max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);

        fl.run(m -> m.setPower(pFL / max));
        fr.run(m -> m.setPower(pFR / max));
        bl.run(m -> m.setPower(pBL / max));
        br.run(m -> m.setPower(pBR / max));

        orchestrator.publish("drive/power/fl", pFL / max);
        orchestrator.publish("drive/power/fr", pFR / max);
        orchestrator.publish("drive/power/bl", pBL / max);
        orchestrator.publish("drive/power/br", pBR / max);
    }
}
