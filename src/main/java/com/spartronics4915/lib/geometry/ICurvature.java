package com.spartronics4915.lib.geometry;

public interface ICurvature<S> extends State<S>
{

    double getCurvature();

    double getDCurvatureDs();
}
