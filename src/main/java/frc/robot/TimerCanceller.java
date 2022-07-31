package frc.robot;

import java.util.*;
import java.lang.*;

public class TimerCanceller extends Thread {
    private boolean isDone = false;
    private double timerInMS;
    private double currentTimeInMS;
    private double timeTarget;

    public TimerCanceller(double milliseconds) {
        this.timerInMS = milliseconds;
        this.currentTimeInMS = System.currentTimeMillis();
        this.timeTarget = currentTimeInMS + timerInMS;
    }

    public void run() {
        isDone = System.currentTimeMillis() >= timeTarget;
    }

    public boolean isDone() {
        return isDone;
    }

    public void resetTime(){
        currentTimeInMS = System.currentTimeMillis();
        timeTarget = currentTimeInMS + timerInMS;
    }
}
