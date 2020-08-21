package com.horse.mpclib.lib.util;

public interface Units<E extends Units> {
    double in(E convertTo, double value);
}
