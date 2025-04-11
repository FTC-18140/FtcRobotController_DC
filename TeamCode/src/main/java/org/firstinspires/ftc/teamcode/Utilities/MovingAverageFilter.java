package org.firstinspires.ftc.teamcode.Utilities;

public class MovingAverageFilter {
    private final int windowSize; // Number of values to average
    private final double[] buffer; // Circular buffer to store values
    private int index; // Current position in buffer
    private double sum; // Running sum of values in buffer
    private int count; // Number of values added so far

    /**
     * Constructor for the moving average filter.
     * @param windowSize The number of values to include in the moving average.
     */
    public MovingAverageFilter(int windowSize) {
        if (windowSize <= 0) {
            throw new IllegalArgumentException("Window size must be positive");
        }
        this.windowSize = windowSize;
        this.buffer = new double[windowSize];
        this.index = 0;
        this.sum = 0.0;
        this.count = 0;
    }

    /**
     * Adds a new sensor value and returns the current moving average.
     * @param value The new sensor value to add.
     * @return The moving average of the current window.
     */
    public double addValue(double value) {
// Add new value to sum
        sum += value;

// If buffer is full, subtract the oldest value before overwriting
        if (count >= windowSize) {
            sum -= buffer[index];
        }

// Store new value in buffer
        buffer[index] = value;

// Move to next position in circular buffer
        index = (index + 1) % windowSize;

// Update count until buffer is full
        if (count < windowSize) {
            count++;
        }

// Return current average
        return sum / count;
    }

    /**
     * Resets the filter to initial state.
     */
    public void reset() {
        index = 0;
        sum = 0.0;
        count = 0;
        for (int i = 0; i < windowSize; i++) {
            buffer[i] = 0.0;
        }
    }

    /**
     * Gets the current window size.
     * @return The window size.
     */
    public int getWindowSize() {
        return windowSize;
    }
}
