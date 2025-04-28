package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LedStrip {
    
    private final AddressableLED m_led = new AddressableLED(2); // PWM port 0
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(174); // fix to length

    public LedStrip () {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    // real timer.  floating point value in seconds.
    public double getCurrentTime() { 
        return Timer.getFPGATimestamp();
    }
    
    // real led painter
    public void repaint() {
        m_led.setData(m_ledBuffer);
    }

    // set real pixels
    public void setPixel(int i, int r, int g, int b) {
        // swap red and green channels in the real implementation
        int red = g;
        int green = r;
        int blue = b;
        m_ledBuffer.setRGB(i, red, green, blue);
    }

    public void setPixel(int i, Color color) {
        int rval = (int) (color.red * 255);
        int gval = (int) (color.green * 255);
        int bval = (int) (color.blue * 255);
        setPixel(i, rval, gval, bval);
    }

    public void setPixelWidth(int refpixel, Color color, int width) {
        int pixel;
        for (int i = 0; i < width; i++) {
            pixel = refpixel + i;
            if (pixel >= getBufferLength()) {
                break;
            }
            setPixel(i, color);
        }
    }
    // set a pixel based on a blend of colors c1 & c2 with a ratio (0..1)
    // which is a percentage of C2.
    public void setPixelMix(int i, Color c1, Color c2, double ratio) {
        if (i < getBufferLength()) {
            double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
            double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
            double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
            setPixel(i, new Color(red, green, blue));
        }
    }

    public int getBufferLength() {
        return m_ledBuffer.getLength();
    }

}
