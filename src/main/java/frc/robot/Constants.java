package frc.robot;

public class Constants{
    // Private constructor since we never want an instance of this to be created.
    private Constants(){

    }
    public static final double INCHES_TO_METERS = 0.0254;
    public static final double DISTANCE_BETWEEN_HATCHES_AND_CARGO_HEIGHTS = 28.0 * INCHES_TO_METERS;
    public static final double HATCH_DISENGAGE_DISTANCE = 5.0 * INCHES_TO_METERS;
    public static final double LIFT_DISTANCE_FROM_GROUND = 22 * INCHES_TO_METERS; // meters
    public static final double LIFT_MAX_HEIGHT = 68 * INCHES_TO_METERS; // meters
    public static final double LIFT_POT_MIN_POSITION_VOLTAGE = 0.723;
    public static final double LIFT_POT_MAX_POSITION_VOLTAGE = 0.339; 
    public static final double LIFT_POT_OFFSET = - LIFT_MAX_HEIGHT * (LIFT_POT_MIN_POSITION_VOLTAGE / (LIFT_POT_MAX_POSITION_VOLTAGE - LIFT_POT_MIN_POSITION_VOLTAGE)) + LIFT_DISTANCE_FROM_GROUND; //meters/volt
    public static final double LIFT_POT_FULL_RANGE =  LIFT_MAX_HEIGHT * (1.0 / (LIFT_POT_MAX_POSITION_VOLTAGE - LIFT_POT_MIN_POSITION_VOLTAGE)); 
    public static final double LIFT_HATCH_POSITION_LOW = (26 + (3.0/8.0)) * INCHES_TO_METERS;
    public static final double LIFT_HATCH_POSITION_MID = LIFT_HATCH_POSITION_LOW + DISTANCE_BETWEEN_HATCHES_AND_CARGO_HEIGHTS;
    public static final double LIFT_HATCH_POSITION_HIGH = LIFT_HATCH_POSITION_MID + DISTANCE_BETWEEN_HATCHES_AND_CARGO_HEIGHTS;
}