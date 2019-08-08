package com.spartronics4915.lib.geometry;

import com.spartronics4915.lib.util.CSVWritable;
import com.spartronics4915.lib.util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable
{

    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
