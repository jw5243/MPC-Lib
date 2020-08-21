package com.horse.mpclib.lib.util;

public interface Interpolable<T> {
    T interpolate(T other, double x);
}
