package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LedSubsystem extends SubsystemBase {

    LedStrip ledStrip;    
    LedMode mode;
    Color primaryColor;
    Color secondaryColor;

    private FireEffect fireLeftEffect;
    private FireEffect fireRightEffect;
    private PacmanEffect pacmanEffect;
    private final int firelow = 7;
    private final int midlow = 64;
    private final int midhigh = 118;
    private final int firehigh = 174;


    public LedSubsystem() {
        ledStrip = new LedStrip();
        mode = LedMode.SOLID;
        primaryColor = Color.kBlack;
        secondaryColor = Color.kBlack;
        
        fireRightEffect = new FireEffect(57, firelow, midlow); // vertical portion of the strip is 57 pixels high
        
        fireLeftEffect = new FireEffect(57, 118, 173);

        pacmanEffect = new PacmanEffect(170);
    }

    // setter for current primary Color
    public void setColor(Color color) {
        primaryColor = color;
    }
    public void setSecondaryColor(Color color) {
        secondaryColor = color;
    }

    // setter for active mode
    public void setMode(LedMode amode) {
        mode = amode;
    }
    
    @Override
    public void periodic() {
        dispatchLedEffect(mode, primaryColor);
        ledStrip.repaint();
        SmartDashboard.putString("led/mode", this.mode.toString());
        SmartDashboard.putString("led/primary_color", this.primaryColor.toString());
        SmartDashboard.putString("led/secondary_color", this.secondaryColor.toString());
    }

    public void dispatchLedEffect(LedMode mode, Color primaryColor) {
        if (mode == LedMode.SOLID) {
            solid(primaryColor);
        } else if (mode == LedMode.STROBE) {
            strobe(primaryColor, 0.1);
        } else if (mode == LedMode.FLASH) {
            strobe(primaryColor, 1.0);
        } else if (mode == LedMode.WAVE) {
            wave(primaryColor, secondaryColor, 25.0, 1.0);  
        } else if (mode == LedMode.WAVE2) {
            wave(primaryColor, secondaryColor, 10.0, 0.5);  
        } else if (mode == LedMode.FIRE) {
            // one-sided fire // 
            //fireEffect.update(ledStrip, Color.WHITE, Color.YELLOW, Color.RED, Color.BLACK);
            solid(Color.kOrange, midlow, midhigh);
            fireLeftEffect.update(ledStrip, Color.kWhite, Color.kYellow, Color.kRed, Color.kBlack, true);
            fireRightEffect.update(ledStrip, Color.kWhite, Color.kYellow, Color.kRed, Color.kBlack, false);
        } else if (mode == LedMode.WATER) {
            // two-sided fire in reef themed colors to look like water//
            solid(Color.kBlue, midlow, midhigh);
            fireLeftEffect.update(ledStrip, Color.kPurple, Color.kGreen, Color.kBlue, Color.kBlack, false);
            fireRightEffect.update(ledStrip, Color.kGreen, Color.kPurple, Color.kBlue, Color.kBlack, true);

        } else if (mode == LedMode.PACMAN) {
            pacmanEffect.update(ledStrip);
        }
   
    }
    

    private void solid(Color color, int low, int high) {
        for (var i = low; i < high; i++) {
            ledStrip.setPixel(i, color);
        }
    }

    private void solid(Color color) {
        if (color != null) {
            for (var i = 0; i < ledStrip.getBufferLength(); i++) {
                ledStrip.setPixel(i, color);
            }
        }
    }

    // two color strobe
    private void strobe(Color c1, Color c2, double duration) {
        boolean c1On = ((ledStrip.getCurrentTime() % duration) / duration) > 0.5;
        solid(c1On ? c1 : c2);
    }
    
    // one color strobe
    private void strobe(Color color, double duration) {
        strobe(color, Color.kBlack, duration);
    }

    private void wave(Color c1, Color c2, double cycleLength, double duration) {
        double waveExponent = 0.4;
        double x = (1 - ((ledStrip.getCurrentTime() % duration) / duration)) * 2.0 * Math.PI;
        double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
        for (var i = 0; i < ledStrip.getBufferLength(); i++) {
            x += xDiffPerLed;
            double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
            if (Double.isNaN(ratio)) {
                ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
            }
            if (Double.isNaN(ratio)) {
                ratio = 0.5;
            }
            ledStrip.setPixelMix(i, c1, c2, ratio);
        }
    }

    public static enum LedMode {
        SOLID,
        STROBE,
        WAVE,
        WAVE2,
        FLASH,
        PACMAN,
        WATER,
        FIRE;
    }
    
}
