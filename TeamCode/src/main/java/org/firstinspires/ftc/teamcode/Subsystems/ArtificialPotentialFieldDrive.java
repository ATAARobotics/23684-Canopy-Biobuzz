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

            // Bottom edge
            for (int i = 0; i <= samplesX; i++) {
                double px = fieldX + (x * i / samplesX);
                double py = fieldY;

                rectangle.add(
                        new FieldBody(x, weight, px, py)
                );
            }

            // Top edge
            for (int i = 0; i <= samplesX; i++) {
                double px = fieldX + (x * i / samplesX);
                double py = fieldY + y;

                rectangle.add(
                        new FieldBody(x, weight, px, py)
                );
            }

            // Left edge
            for (int i = 1; i < samplesY; i++) {
                double px = fieldX;
                double py = fieldY + (y * i / samplesY);

                rectangle.add(
                        new FieldBody(y, weight, px, py)
                );
            }

            // Right edge
            for (int i = 1; i < samplesY; i++) {
                double px = fieldX + x;
                double py = fieldY + (y * i / samplesY);

                rectangle.add(
                        new FieldBody(y, weight, px, py)
                );
            }
        }

        public double repulsiveForce() {
            forces.clear();

            for (FieldBody fieldBody : rectangle) {
                forces.add(fieldBody.repulsiveForce());
            }

            if (forces.isEmpty()) {
                return 0;
            }

            return Collections.max(forces);
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

        public double repulsiveForce(){
            double G = 6.67;
            double robotM = 15;
            //return -(G*((robotM + weight)/(DistanceFromDia(5,5)))); //TODO: Make this be safepedro Pose
            double fin;

            if((0.5*(DistanceFromDia(5,5))-2.5) < 0){
                fin = 0.5*(DistanceFromDia(5,5))-2.5;
            }else{
                fin = 0;
            }

            return fin;
        }

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
