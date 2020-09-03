package com.horse.mpclib.lib.util;

import android.os.SystemClock;

public class TimeUtil {
    public static boolean isUsingComputer = true;

    private static final double SECONDS_PER_MINUTE = 60d;
    private static final double MILLISECONDS_PER_SECOND = 1000d;
    private static final double NANOSECONDS_PER_MILLISECOND = 1000d;
    private static long startTime;

    public static void startTime() {
        setStartTime(getAbsoluteTimeMilliseconds());
    }

    public static double getCurrentRuntime(final TimeUnits units) {
        return (int)(1000d * TimeUnits.MILLISECONDS.in(units, getAbsoluteTimeMilliseconds() - getStartTime())) / 1000d;
    }

    public static Time getCurrentRuntime() {
        return new Time(getCurrentRuntime(TimeUnits.MILLISECONDS), TimeUnits.MILLISECONDS);
    }

    public static long getAbsoluteTimeMilliseconds() {
        return isUsingComputer ? System.currentTimeMillis() : SystemClock.uptimeMillis();
    }

    public static double getSecondsPerMinute() {
        return SECONDS_PER_MINUTE;
    }

    public static double getMillisecondsPerSecond() {
        return MILLISECONDS_PER_SECOND;
    }

    public static double getNanosecondsPerMillisecond() {
        return NANOSECONDS_PER_MILLISECOND;
    }

    public static long getStartTime() {
        return startTime;
    }

    public static void setStartTime(long startTime) {
        TimeUtil.startTime = startTime;
    }
}
