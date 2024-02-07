package org.firstinspires.ftc.teamcode.filters;

import java.util.ArrayList;
import java.util.Collections;

public class MedianFilter {

    private int window = 0;
    private ArrayList<Double> prevVals;

    public MedianFilter(int window) {
        this.window = window;
        this.prevVals = new ArrayList<Double>();
    }

    private double getMedian(ArrayList<Double> vals) {
        Collections.sort(vals);
        double middle = vals.size()/2;
        if (vals.size()%2 == 0) {
            middle = (vals.get(vals.size()/2) + vals.get(vals.size()/2 - 1))/2;
        } else {
            middle = vals.get(vals.size() / 2);
        }
        return middle;
    }

    public double calculate(double input) {
        prevVals.add(input);
        if (prevVals.size() > window) {
            prevVals.remove(0);
        }

        return getMedian(prevVals);
    }
}
