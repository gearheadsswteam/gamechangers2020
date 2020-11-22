package org.firstinspires.ftc.teamcode.robot.utils;

public class Movement {
    Position initial;
    Position target;
    double speed = 1;

    public Movement(Position initial, Position target) {
        this.initial = initial;
        this.target = target;
    }

    public Movement(Position initial, Position target, double speed) {
        this.initial = initial;
        this.target = target;
        this.speed = speed;
    }

    public double getDistance(){
        return Math.hypot(getX(), getY());
    }

    public float getTargetOrientation(){
        return getY()/getY();
    }

    public float getX(){
        return (target.x - initial.x);
    }


    public float getY(){
        return (target.y - initial.y);
    }

    public double getNormalizedX(){
        float dx = target.x - initial.x;
        float dy = target.y - initial.y;

        float max = Math.max(Math.abs(dx), Math.abs(dy));

        return (dx/max)*speed;
    }


    public double getNormalizedY(){
        float dx = target.x - initial.x;
        float dy = target.y - initial.y;

        float max = Math.max(Math.abs(dx), Math.abs(dy));

        return (dy/max)*speed;
    }


}
