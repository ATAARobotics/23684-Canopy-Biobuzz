package org.firstinspires.ftc.teamcode.HiveVision;

import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.math.Vector2D;
import com.skeletonarmy.marrow.zones.Point;

import org.firstinspires.ftc.teamcode.Subsystems.SafePedroPathingConverter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

public class CVCostAlgo {
    private ArrayList<ArrayList<Pose>> permutationsGen(ArrayList<Pose> ballList, ArrayList<Pose> buffer, ArrayList<ArrayList<Pose>> candidates, int index){

        if(buffer.size() == Math.min(4, ballList.size())){
            candidates.add(new ArrayList<>(buffer));
            return candidates;
        }

        for(int i = index; i < ballList.size(); i++){
            buffer.add(ballList.get(i));

            permutationsGen(ballList, buffer, candidates, i + 1);

            buffer.remove(buffer.size() - 1);
        }
        return candidates;
    }
    public ArrayList<Point> costFunc(ArrayList<Pose> balls, Pose curpose){
        if (balls.isEmpty()) {
            return new ArrayList<>();
        }
        ArrayList<ArrayList<Pose>> canidateGroups = new ArrayList<>();
        canidateGroups.addAll(permutationsGen(balls, new ArrayList<>(), new ArrayList<>(), 0));
        ArrayList<ArrayList<Pose>> candidateRoutes = new ArrayList<>();

        for(ArrayList<Pose> group : canidateGroups) {
            ArrayList<Pose> sorted = new ArrayList<>();
            group.sort(Comparator.comparingDouble(ball -> curpose.distance(ball)));
            sorted.add(group.remove(0));
            while (!group.isEmpty()) {
                group.sort(Comparator.comparingDouble(ball -> sorted.get(sorted.size() - 1).distance(ball)));
                sorted.add(group.remove(0));
            }
            candidateRoutes.add(sorted);
        }
        ArrayList<Routes> routes = new ArrayList<>();
        for (ArrayList<Pose> group : candidateRoutes) {
            ArrayList<Pose> route = new ArrayList<>(group.subList(0, Math.min(group.size(), 4)));
            double dist = curpose.distance(route.get(0));
            for (int i = 0; i < route.size() - 1; i++) {
                dist += route.get(i).distance(route.get(i + 1));
            }
            routes.add(new Routes(route, dist));
        }

        routes.sort(Comparator.comparingDouble(route -> route.score));
        Collections.reverse(routes);

        return wallPadding(routes.get(0).balls, curpose);


    }
    public ArrayList<Point> wallPadding (ArrayList<Pose> balls, Pose curPose){
        ArrayList<Point> bordered = new ArrayList<>();
        double padding = 7;
        for(Pose ball : balls){
            if(ball.x() <= padding) {
                ball = ball.withX(padding);
            }else if(ball.x() >= 141.5-padding){
                ball = ball.withX(141.5-padding);
            }
            if(ball.y() <= 7){
                ball = ball.withY(padding);
            } else if(ball.y() >= 141.5-padding){
                ball = ball.withY(141.5-padding);
            }
            bordered.add(new Point(ball.x(),ball.y()));
        }
        return bordered;
    }
    public ArrayList<Point> combiner(ArrayList<Point> route, Pose curPose){
        if(route.size() <= 1) return route;
        for(int i = 0; i < route.size() - 1; i++){
            Pose root = new Pose(route.get(i).getX(), route.get(i).getY());
            Pose nextroot = new Pose(route.get(i+1).getX(), route.get(i+1).getY());
            if(root.distance(nextroot) > 16) continue;

            Vector2D ballToBall = nextroot.toVector2D().minus(root.toVector2D());

            double theta;
            if(i == 0) {
                theta = Math.atan2(root.y() - curPose.y(), root.x() - curPose.x());
            }else{
                Pose lastroot = new Pose(route.get(i - 1).getX(), route.get(i - 1).getY());

                theta = Math.atan2(root.y() - lastroot.y(), root.x() - lastroot.x());
            }


            Vector2D botVec = Vector2D.polar(1, theta + Math.PI/2.0);
            double compx = botVec.dot(ballToBall);

            Vector2D botVecPerp = Vector2D.polar(1, theta);
            double compy = botVecPerp.dot(ballToBall);
            if(Math.abs(compx) < 10 && Math.abs(compy) < 6){
                double x = (root.x() + nextroot.x()) / 2.0;
                double y = (root.y() + nextroot.y()) / 2.0;
                route.set(i,new Point(x, y));
                route.remove(i+1);
            }
        }
        return route;
    }
    class Routes {
        ArrayList<Pose> balls;
        double distance;
        double score;
        Routes(ArrayList<Pose> balls, double distance) {
            this.balls = balls;
            this.distance = distance;
            this.score = balls.size() / distance;
        }
    }
}
