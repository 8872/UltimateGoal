package org.firstinspires.ftc.teamcode.util;

import java.util.concurrent.TimeUnit;

public class Timer {

    private final long duration;
    private long startTime;

    public Timer(long durationMillis) {
        this(durationMillis, TimeUnit.MILLISECONDS);
    }

    public Timer(long duration, TimeUnit unit) {
        if (unit.compareTo(TimeUnit.MILLISECONDS) < 0) {
            throw new IllegalArgumentException("Time unit cannot be lower than milliseconds");
        }
        this.duration = unit.toMillis(duration);
    }

    public void reset() {
        startTime = 0;
    }

    public void start() {
        startTime = System.currentTimeMillis();
    }

    public boolean isStarted() {
        return startTime != 0;
    }

    public long elapsedMillis() {
        return System.currentTimeMillis() - startTime;
    }

    public boolean isFinished() {
        return isStarted() && elapsedMillis() >= duration;
    }
}
