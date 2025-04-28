package frc.robot.subsystems.led;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Random;


public class FireEffect {
    private byte[] heat;
    private Random random;
    private int cooling;
    private int sparking;

    private int numLeds;
    private int low;
    private int high;
    //private boolean reverse;
    
    public FireEffect(int numLeds, int low, int high) {
        // seed the random number so if we instantiate two of these they look symmetric
        random = new Random(12345);
        this.low = low;
        this.high = high;
        //this.reverse = reverse;
        this.numLeds = numLeds;
        heat = new byte[numLeds];
        cooling = 80; //55;
        sparking = 120;
    }

    public void update(LedStrip ledStrip, Color hot2, Color hot1, Color hot0, Color dark, boolean reverse) {
        int cooldown;

        // Step 1. Cool down every cell a little
        for (int i = 0; i < numLeds; i++) {
            cooldown = random.nextInt(((cooling * 10) / numLeds) + 2);

            if ((heat[i] & 0xFF) <= cooldown) { // Compare unsigned values
                heat[i] = 0;
            } else {
                heat[i] = (byte) ((heat[i] & 0xFF) - cooldown); // Subtract unsigned, store signed
            }
        }

        // Step 2. Heat from each cell drifts 'up' and diffuses a little
        for (int k = numLeds - 1; k >= 2; k--) {
            int h1 = heat[k - 1] & 0xFF;
            int h2 = heat[k - 2] & 0xFF;
            heat[k] = (byte) ((h1 + h2 + h2) / 3);
        }

        // Step 3. Randomly ignite new 'sparks' near the bottom
        if (random.nextInt(256) < sparking) {
            int y = random.nextInt(7);
            heat[y] = (byte) (random.nextInt(160, 256)); // Directly assign unsigned value
        }

        // Step 4. Convert heat to LED colors
        for (int i = 0; i < numLeds; i++) {
            int pixel;
            if (reverse) {
                pixel = high - i;
            } else {
                pixel = low + i;
            }
            assert(pixel >= 0);
            assert(pixel < numLeds);

            byte temperature = heat[i];
            // Scale 'heat' down from 0-255 to 0-191 (using unsigned value)
            int tempUnsigned = temperature & 0xFF;
            int t192 = (int) Math.round((tempUnsigned / 255.0) * 191);

            // calculate ramp up
            int heatramp = t192 & 0x3F; // 0..63
            double heatratio = heatramp / 63.0;

            // figure out which third of the spectrum we're in:
            if (t192 > 0x80) { // hottest
                ledStrip.setPixelMix(pixel, hot1, hot2, heatratio);
            } else if (t192 > 0x40) { // middle
                ledStrip.setPixelMix(pixel, hot0, hot1, heatratio);
            } else { // coolest
                ledStrip.setPixelMix(pixel, dark, hot0, heatratio);
            }
        }
    }
}

