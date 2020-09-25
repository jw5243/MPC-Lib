package com.horse.mpclib.lib.controllers;

public enum ControllerBehavior {
    AGRESSIVE(1000d), STANDARD(100d), SLOW(10d);

    final double costFactor;

    ControllerBehavior(final double costFactor) {
        this.costFactor = costFactor;
    }

    public double getCostFactor() {
        return costFactor;
    }
}
