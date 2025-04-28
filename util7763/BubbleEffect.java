import java.util.Random;
import java.awt.Color;

public class BubbleEffect {
    private static final int NUM_BUBBLES = 5;
    private Bubble[] bubbles;
    private Random random;
    private Color color1;
    private Color color2;
    private Color color3;
    private Color darkColor;
    private int numLeds;

    private class Bubble {
        public double position;
        public int size;
        public double speed;
        public Color color;

        public Bubble(double position, int size, double speed, Color color) {
            this.position = position;
            this.size = size;
            this.speed = speed;
            this.color = color;
        }
    }

    public BubbleEffect(int numLeds, Color color1, Color color2, Color color3, Color darkColor) {
        this.numLeds = numLeds;
        this.color1 = color1;
        this.color2 = color2;
        this.color3 = color3;
        this.darkColor = darkColor;
        this.random = new Random();
        this.bubbles = new Bubble[NUM_BUBBLES];
        initializeBubbles();
    }

    private void initializeBubbles() {
        for (int i = 0; i < NUM_BUBBLES; i++) {
            createNewBubble(i);
        }
    }

    private void createNewBubble(int index) {
        int size = random.nextInt(8) + 3; // Size between 3 and 10
        double speed = 0.5 + (0.05 * size); // Speed proportional to size
        double startPosition = -size; // Start above the strip
        Color bubbleColor;
        int colorSelector = random.nextInt(3);
        if (colorSelector == 0) {
            bubbleColor = color1;
        } else if (colorSelector == 1) {
            bubbleColor = color2;
        } else {
            bubbleColor = color3;
        }
        bubbles[index] = new Bubble(startPosition, size, speed, bubbleColor);
    }

    public void update(LedStrip ledStrip) {
        // Clear the strip to the dark color
        for (int i = 0; i < numLeds; i++) {
            ledStrip.setPixel(i, darkColor);
        }

        // Update and draw each bubble
        for (int i = 0; i < NUM_BUBBLES; i++) {
            updateBubble(bubbles[i]);
            drawBubble(ledStrip, bubbles[i]);
        }
    }

    private void updateBubble(Bubble bubble) {
        bubble.position += bubble.speed;
        if (bubble.position - bubble.size > numLeds) {
            // Bubble has left the strip, create a new one
            createNewBubble(getBubbleIndex(bubble));
        }
    }
    
    private int getBubbleIndex(Bubble bubble) {
        for (int i = 0; i < bubbles.length; i++) {
            if (bubbles[i] == bubble) {
                return i;
            }
        }
        return -1; // Should not happen
    }

    private void drawBubble(LedStrip ledStrip, Bubble bubble) {
        int center = (int) Math.round(bubble.position);
        for (int i = -bubble.size; i <= bubble.size; i++) {
            int pixel = center + i;
            if (pixel >= 0 && pixel < numLeds) {
                double distance = Math.abs(i);
                double ratio = 1.0;// - (distance / bubble.size*2); // Dimming factor
                if (ratio < 0) ratio = 0;
                Color dimmedColor = dimColor(bubble.color, ratio);
                
                // Blend with existing color
                //Color existingColor = ledStrip.getPixelColor(pixel);
                //Color blendedColor = blendColors(existingColor, dimmedColor);
                ledStrip.setPixel(pixel, dimmedColor);
            }
        }
    }

    private Color dimColor(Color color, double ratio) {
        double red = color.getRed() * ratio;
        double green = color.getGreen() * ratio;
        double blue = color.getBlue() * ratio;
        return new Color((int)red, (int)green, (int)blue);
    }

    // private Color blendColors(Color color1, Color color2) {
    //     double red = (color1.red + color2.red) / 2.0;
    //     double green = (color1.green + color2.green) / 2.0;
    //     double blue = (color1.blue + color2.blue) / 2.0;
    //     return new Color(red, green, blue);
    // }
}
