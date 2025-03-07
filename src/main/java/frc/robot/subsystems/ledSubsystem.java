package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ledSubsystem extends SubsystemBase{

    private static final double waveExponent = 0.4;

    private final AddressableLED m_led = new AddressableLED(0); // PWM port 0
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(300); // fix to length

    Timer ledTimerOn;
    Timer ledTimerOff;
    int red, green, blue;
    Color saveColor;
    LedMode mode;

    public ledSubsystem() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
        ledTimerOff = new Timer();
        ledTimerOn = new Timer();
        ledTimerOn.start();
        ledTimerOff.reset();
        ledTimerOn.reset();
        mode = LedMode.SOLID;
        saveColor = Color.kBlack;
        red = 0;
        green = 0;
        blue = 0;
    }

    private void setBufferIndexRGB(int i, int r, int g, int b) {
        red = g;
        green = r;
        blue = b;
        m_ledBuffer.setRGB(i, red, green, blue);
    }

    public void setBufferIndexColor(int i, Color color) {
        int rval = (int) (color.red * 255);
        int gval = (int) (color.green * 255);
        int bval = (int) (color.blue * 255);
        setBufferIndexRGB(i, rval, gval, bval);
    }

    public void setColor(Color color) {
        saveColor = color;
    }
    
    public void setMode(LedMode amode) {
        mode = amode;
    }
    
    @Override
    public void periodic() {
        if (mode == LedMode.SOLID) {
            solid(saveColor);
        } else if (mode == LedMode.STROBE) {
            strobe(saveColor, 0.1);
        } else if (mode == LedMode.FLASH) {
            strobe(saveColor, 1.0);
        } else if (mode == LedMode.WAVE) {
            wave(saveColor, Color.kBlack, 25.0, 1.0);
        }
        m_led.setData(m_ledBuffer);

        SmartDashboard.putString("led/colorRgb", this.saveColor.toString());
        SmartDashboard.putString("led/mode", this.mode.toString());

        //Timer.getFPGATimestamp()
        // if (blink){
        //     if (ledTimerOn.get() >= .15){
        //         ledTimerOn.reset();
        //         ledTimerOn.stop();
        //         ledTimerOff.start();
        //         setRGB(red, green, blue);
        //     }
        //     else if (ledTimerOff.get() >= .15){
        //         ledTimerOff.reset();
        //         ledTimerOff.stop();
        //         ledTimerOn.start();
        //         setColor(Color.kBlack);
        //     }
        // }
    }

    // Set to color
    public void setColors(Color color, Color color2) {
        var m_ledCarbo = new Color();
        m_ledCarbo = color;
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            if (i % 20 == 0){
                if (m_ledCarbo == color){
                    m_ledCarbo = color2;
                }
                else{
                    m_ledCarbo = color;
                }
            }
            m_ledBuffer.setLED(i, m_ledCarbo);
        }
        m_led.setData(m_ledBuffer);
    }

    private void solid(Color color) {
        if (color != null) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                setBufferIndexColor(i, color);
            }
        }
    }

    // two color strobe
    private void strobe(Color c1, Color c2, double duration) {
        boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
        solid(c1On ? c1 : c2);
    }
    
    // one color strobe
    private void strobe(Color color, double duration) {
        strobe(color, Color.kBlack, duration);
    }

    private void wave(Color c1, Color c2, double cycleLength, double duration) {
        double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
        double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          x += xDiffPerLed;
          double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
          if (Double.isNaN(ratio)) {
            ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
          }
          if (Double.isNaN(ratio)) {
            ratio = 0.5;
          }
          double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
          double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
          double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
          setBufferIndexColor(i, new Color(red, green, blue));
        }
      }
    

    public static enum LedMode {
        SOLID,
        STROBE,
        WAVE,
        FLASH;
    }
    
}
