package frc.team4481.robot.subsystems.output;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class BlinkinLED
{
    private Spark LEDDriver;
    public BlinkinLED(int ID)
    {
        //TODO kijken of dit werkt
        LEDDriver = new Spark(ID);
        LEDDriver.isAlive();
        enable();
    }

    public void enable()
    {
        LEDDriver.set(Pattern.SOLID_COLOR_WHITE.value);
    }

    public void setPattern(Pattern p)
    {
        LEDDriver.set(p.value);
    }

    public void set(double p) {LEDDriver.set(p);}

    public void disable()
    {
        LEDDriver.set(0.00);
    }

    public void setHoldCone() {LEDDriver.set(Pattern.SOLID_COLOR_YELLOW.value); }
    public void setIntakingCone() {LEDDriver.set(Pattern.C1_STROBE.value);}
    public void setHoldCube() {LEDDriver.set(Pattern.SOLID_COLOR_VIOLET.value); }
    public void setIntakingCube() {LEDDriver.set(Pattern.C2_STROBE.value);}

    @SuppressWarnings("unused")
    public enum Pattern
    {
        /* Different color palettes */

        RAINBOW_RAINBOW_PALETTE(-0.99),
        RAINBOW_PARTY_PALETTE(-0.97),
        RAINBOW_OCEAN_PALETTE(-0.95),
        RAINBOW_LAVE_PALETTE(-0.93),
        RAINBOW_FOREST_PALETTE(-0.91),
        RAINBOW_GLITTER(-0.89),

        CONFETTI(-0.87),

        SHOT_RED(-0.85),
        SHOT_BLUE(-0.83),
        SHOT_WHITE(-0.81),

        SINELON_RAINBOW_PALETTE(-0.79),
        SINELON_PARTY_PALETTE(-0.77),
        SINELON_OCEAN_PALETTE(-0.75),
        SINELON_LAVA_PALETTE(-0.73),
        SINELON_FOREST_PALETTE(-0.71),

        BPM_RAINBOW_PALETTE(-0.69),
        BPM_PARTY_PALETTE(-0.67),
        BPM_OCEAN_PALETTE(-0.65),
        BPM_LAVA_PALETTE(-0.63),
        BPM_FOREST_PALETTE(-0.61),

        FIRE_MEDIUM(-0.59),
        FIRE_LARGE(-0.57),

        TWINKLES_RAINBOW_PALETTE(-0.55),
        TWINKLES_PARTY_PALETTE(-0.53),
        TWINKLES_OCEAN_PALETTE(-0.51),
        TWINKLES_LAVA_PALETTE(-0.49),
        TWINKLES_FOREST_PALETTE(-0.47),

        COLOR_WAVES_RAINBOW_PALETTE(-0.45),
        COLOR_WAVES_PARTY_PALETTE(-0.43),
        COLOR_WAVES_OCEAN_PALETTE(-0.41),
        COLOR_WAVES_LAVA_PALETTE(-0.39),
        COLOR_WAVES_FOREST_PALETTE(-0.37),

        LARSON_SCANNER_RED(-0.35),
        LARSON_SCANNER_GRAY(-0.33),

        LIGHT_CHASE_RED(-0.31),
        LIGHT_CHASE_BLUE(-0.29),
        LIGHT_CHASE_GRAY(-0.27),

        HEARTBEAT_RED(-0.25),
        HEARTBEAT_BLUE(-0.23),
        HEARTBEAT_WHITE(-0.21),
        HEARTBEAT_GRAY(-0.19),

        BREATH_CHASE_RED(-0.17),
        BREATH_CHASE_BLUE(-0.15),
        BREATH_CHASE_GRAY(-0.13),

        STROBE_RED(-0.11),
        STROBE_BLUE(-0.09),
        STROBE_GOLD(-0.07),
        STROBE_WHITE(-0.05),

        /* color patterns set on Blinkin LED driver */

        C1_PATTERN_END_TO_END_BLEND_TO_BLACK(-0.03),
        C1_PATTERN_LARSON_SCANNER(-0.01),
        C1_LIGHT_CHASE(0.01),
        C1_HEARTBEAT_SLOW(0.03),
        C1_HEARTBEAT_MEDIUM(0.05),
        C1_HEARTBEAT_FAST(0.07),
        C1_BREATH_SLOW(0.09),
        C1_BREATH_FAST(0.11),
        C1_SHOT(0.13),
        C1_STROBE(0.15),

        C2_PATTERN_END_TO_END_BLEND_TO_BLACK(0.17),
        C2_PATTERN_LARSON_SCANNER(0.19),
        C2_LIGHT_CHASE(0.21),
        C2_HEARTBEAT_SLOW(0.23),
        C2_HEARTBEAT_MEDIUM(0.35),
        C2_HEARTBEAT_FAST(0.27),
        C2_BREATH_SLOW(0.29),
        C2_BREATH_FAST(0.31),
        C2_SHOT(0.33),
        C2_STROBE(0.35),

        C1_C2_PATTERN_SPARKLE_C1_ON_C2(0.37),
        C1_C2_PATTERN_SPARKLE_C2_ON_C1(0.39),
        C1_C2_PATTERN_COLOR_GRADIENT_C1_ON_C2(0.41),
        C1_C2_PATTERN_BPM(0.43),
        C1_C2_PATTERN_END_TO_END_BLEND_C1_TO_C2(0.45),
        C1_C2_PATTERN_END_TO_END_BLEND(0.47),
        C1_C2_PATTERN_NO_BLEND_C1_C2(0.49),
        C1_C2_PATTERN_TWINKLES(0.51),
        C1_C2_PATTERN_COLOR_WAVES(0.53),
        C1_C2_PATTERN_SINELON(0.55),

        /* Solid colors */

        SOLID_COLOR_HOT_PINK(0.57),
        SOLID_COLOR_DARK_RED(0.59),
        SOLID_COLOR_RED(0.61),
        SOLID_COLOR_RED_ORANGE(0.63),
        SOLID_COLOR_ORANGE(0.65),
        SOLID_COLOR_GOLD(0.67),
        SOLID_COLOR_YELLOW(0.69),
        SOLID_COLOR_LAWN_GREEN(0.71),
        SOLID_COLOR_LIME(0.73),
        SOLID_COLOR_DARK_GREEN(0.75),
        SOLID_COLOR_GREEN(0.77),
        SOLID_COLOR_BLUE_GREEN(0.79),
        SOLID_COLOR_AQUA(0.81),
        SOLID_COLOR_SKY_BLUE(0.83),
        SOLID_COLOR_DARK_BLUE(0.85),
        SOLID_COLOR_BLUE(0.87),
        SOLID_COLOR_BLUE_VIOLET(0.89),
        SOLID_COLOR_VIOLET(0.91),
        SOLID_COLOR_WHITE(0.93),
        SOLID_COLOR_GRAY(0.95),
        SOLID_COLOR_DARK_GRAY(0.97),
        SOLID_COLOR_BLACK(0.99);

        double value;

        Pattern (double value)
        {
            this.value = value;
        }
    }

}
