package com.horse.mpclib.lib.geometry;

import com.horse.mpclib.lib.util.Interpolable;

public interface State<S> extends Interpolable<S> {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();
}
