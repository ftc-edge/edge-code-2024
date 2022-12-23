/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package multiteams;

import TrcCommonLib.trclib.TrcRevBlinkin;
import TrcFtcLib.ftclib.FtcRevBlinkin;

/**
 * This class encapsulates the REV Blinkin LED controller to provide a priority indicator showing the status of the
 * robot.
 */
class BlinkinLEDs extends FtcRevBlinkin
{
    // LED pattern names.
    public static final String LABEL_TARGET1 = "Target1";
    public static final String LABEL_TARGET2 = "Target2";
    public static final String LABEL_TARGET3 = "Target3";
    public static final String DETECTED_RED_BLOB = "RedBlob";
    public static final String DETECTED_BLUE_BLOB = "BlueBlob";
    public static final String DETECTED_YELLOW_BLOB = "YellowBlob";
    public static final String IMAGE1_NAME = "Image1";
    public static final String IMAGE2_NAME = "Image2";
    public static final String IMAGE3_NAME = "Image3";
    public static final String IMAGE4_NAME = "Image4";
    public static final String DRIVE_ORIENTATION_FIELD = "FieldMode";
    public static final String DRIVE_ORIENTATION_ROBOT = "RobotMode";
    public static final String DRIVE_ORIENTATION_INVERTED = "InvertedMode";

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name. This is also the REV Blinkin's hardware name.
     */
    public BlinkinLEDs(String instanceName)
    {
        super(instanceName);
        // LED pattern priority list in decreasing priority order.
        final TrcRevBlinkin.Pattern[] ledPatternPriorities = {
            new TrcRevBlinkin.Pattern(LABEL_TARGET1, TrcRevBlinkin.RevLedPattern.SolidRed),
            new TrcRevBlinkin.Pattern(LABEL_TARGET2, TrcRevBlinkin.RevLedPattern.SolidGreen),
            new TrcRevBlinkin.Pattern(LABEL_TARGET3, TrcRevBlinkin.RevLedPattern.SolidBlue),
            new TrcRevBlinkin.Pattern(DETECTED_RED_BLOB, TrcRevBlinkin.RevLedPattern.FixedStrobeRed),
            new TrcRevBlinkin.Pattern(DETECTED_BLUE_BLOB, TrcRevBlinkin.RevLedPattern.FixedStrobeBlue),
            new TrcRevBlinkin.Pattern(DETECTED_YELLOW_BLOB, TrcRevBlinkin.RevLedPattern.FixedStrobeGold),
            new TrcRevBlinkin.Pattern(IMAGE1_NAME, TrcRevBlinkin.RevLedPattern.FixedBreathRed),
            new TrcRevBlinkin.Pattern(IMAGE2_NAME, TrcRevBlinkin.RevLedPattern.FixedBreathBlue),
            new TrcRevBlinkin.Pattern(IMAGE3_NAME, TrcRevBlinkin.RevLedPattern.FixedLightChaseRed),
            new TrcRevBlinkin.Pattern(IMAGE4_NAME, TrcRevBlinkin.RevLedPattern.FixedLightChaseBlue),
            new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_FIELD, TrcRevBlinkin.RevLedPattern.SolidViolet),
            new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_ROBOT, TrcRevBlinkin.RevLedPattern.SolidWhite),
            new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_INVERTED, TrcRevBlinkin.RevLedPattern.SolidGray)
        };
        setPatternPriorities(ledPatternPriorities);
    }   //BlinkinLEDs

}   //class BlinkinLEDs
