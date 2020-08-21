package com.horse.mpclib.debugging;

import com.horse.mpclib.lib.geometry.Pose2d;

public interface RobotDebug {
    void init_debug();

    void start_debug();

    void loop_debug() throws IllegalMessageTypeException;

    void sendMotionProfileData();

    Pose2d getFieldPosition();
}
