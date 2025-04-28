
import java.util.Random;
import javax.swing.*;

import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class LedStripDemo {

    private LedStrip ledStrip; 
    private Random random = new Random();
    private Timer timer;
    private double time = 0;
    private LedMode mode;
    private Color primaryColor;
    private Color secondaryColor;

    // the Effects
    private FireEffect fireEffect;
    private FireEffect fireLeftEffect;
    private FireEffect fireRightEffect;
    private BubbleEffect bubbleEffect;
    private PacmanEffect pacmanEffect;

    public LedStripDemo(LedMode mode, Color primaryColor, Color secondaryColor) {
        /*-----------------------------------------------------------*/
        // these are set from arguments now.
        this.mode = mode;
        this.primaryColor = primaryColor;
        this.secondaryColor = secondaryColor;
        /*-----------------------------------------------------------*/

        ledStrip = new LedStrip();
        int numLeds = ledStrip.getBufferLength();

        // effects
        fireEffect = new FireEffect(numLeds, 0, numLeds-1, false);
        fireRightEffect = new FireEffect(numLeds/2, 0, numLeds/2-1, true);
        fireLeftEffect = new FireEffect(numLeds/2, numLeds/2, numLeds-1, false);
        bubbleEffect = new BubbleEffect(numLeds, Color.GREEN, Color.CYAN, Color.BLUE, Color.BLACK);
        //bubbleRightEffect = new BubbleEffect(numLeds/2, Color.GREEN, Color.CYAN, Color.BLUE, Color.BLACK);

        pacmanEffect = new PacmanEffect(ledStrip.getBufferLength());

        // 20ms to match Timer in real roboRio
        timer = new Timer(20, new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                //dispatchLedEffect(mode, saveColor);
                dispatchLedEffect();
                ledStrip.repaint();
                time += 0.02; // Increment time by 20ms
                ledStrip.setTime(time);

            }
        });
        timer.start();
        
    }

    // display demo gui (MAIN)
    public static void main(String[] args) {
        // Default values if no arguments are provided
        LedMode mode = LedMode.FIRE2;
        Color primaryColor = Color.RED;
        Color secondaryColor = Color.GREEN;

        // Parse arguments if they exist
        if (args.length >= 1) {
            try {
                mode = LedMode.valueOf(args[0].toUpperCase());
            } catch (IllegalArgumentException e) {
                System.err.println("Invalid LedMode: " + args[0] + ". Using default: PACMAN");
            }
        }
        if (args.length >= 2) {
            try {
                primaryColor = argparseColor(args[1]);
            } catch (NumberFormatException e) {
                System.err.println("Invalid primary color: " + args[1] + ". Using default: RED");
            }
        }
            if (args.length >= 3) {
            try {
                secondaryColor = argparseColor(args[2]);
            } catch (NumberFormatException e) {
                System.err.println("Invalid secondary color: " + args[2] + ". Using default: GREEN");
            }
        }

        // Create and run the LedStripDemo with the parsed arguments
        final LedMode finalMode = mode;
        final Color finalPrimaryColor = primaryColor;
        final Color finalSecondaryColor = secondaryColor;
        SwingUtilities.invokeLater(() -> new LedStripDemo(finalMode, finalPrimaryColor, finalSecondaryColor));
    }

       // Helper method to parse a color string, e.g. "RED"
       private static Color argparseColor(String colorString) {
        try {
            return (Color) Color.class.getField(colorString.toUpperCase()).get(null);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new NumberFormatException("Invalid color: " + colorString);
        }
    }

    private void waveEffectRedBlue() {
        for (int i = 0; i < ledStrip.getBufferLength(); i++) {
            double wave = Math.sin((i / (double) ledStrip.getBufferLength() * Math.PI * 2) + time);
            int red = (int) (127.5 * (1 + Math.sin(wave + time)));
            int blue = (int) (127.5 * (1 + Math.cos(wave + time)));
            ledStrip.setPixel(i, red, 0, blue);
        }
    }


    public void dispatchLedEffect() {
        dispatchLedEffect(this.mode, this.primaryColor, this.secondaryColor);
    }

    public void dispatchLedEffect(LedMode mode, Color primaryColor, Color secondaryColor) {
        if (mode == LedMode.SOLID) {
            solid(primaryColor);
        } else if (mode == LedMode.STROBE) {
            strobe(primaryColor, secondaryColor, 0.1);
        } else if (mode == LedMode.FLASH) {
            strobe(primaryColor, secondaryColor, 1.0);
        } else if (mode == LedMode.WAVE) {
            waveEffect(primaryColor, secondaryColor, 25.0, 1.0);
        } else if (mode == LedMode.WAVERB) {
            waveEffectRedBlue();
        } else if (mode == LedMode.FIRE) {
            // one-sided fire // 
            fireEffect.update(ledStrip, Color.WHITE, Color.YELLOW, Color.RED, Color.BLACK);
        } else if (mode == LedMode.FIRE2) {
            // two sided fire //
            fireLeftEffect.update(ledStrip, Color.GREEN, Color.CYAN, Color.BLUE, Color.BLACK);
            fireRightEffect.update(ledStrip, Color.GREEN, Color.CYAN, Color.BLUE, Color.BLACK);            
        } else if (mode == LedMode.BUBBLE) {
            // one-sided fire // fireEffect.update(ledStrip, Color.WHITE, Color.YELLOW, Color.RED, Color.BLACK);
            bubbleEffect.update(ledStrip);
            
        } else if (mode == LedMode.PACMAN) {
             pacmanEffect.update(ledStrip);
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
        strobe(color, Color.BLACK, duration);
    }

    private void waveEffect(Color c1, Color c2, double cycleLength, double duration) {
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
        WAVERB,
        FIRE,
        FIRE2,
        PACMAN,
        BUBBLE,
        FLASH;
    }
    
}
