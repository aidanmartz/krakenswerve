
import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class LedStrip  extends JFrame {

    private static final int LED_COUNT = 120;
    private static final int LED_SIZE = 10;
    private static final int LED_SPACING = 1;
    private static final int WINDOW_WIDTH = LED_COUNT * (LED_SIZE + LED_SPACING) + LED_SPACING;
    private static final int WINDOW_HEIGHT = LED_SIZE + 2 * LED_SPACING + 50;
    private JPanel ledStrip;
    private Color[] ledColors;
    private double time;
    
    public LedStrip() {
        setTitle("LED Strip Display");
        setSize(WINDOW_WIDTH, WINDOW_HEIGHT);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLocationRelativeTo(null);
        time = 0.0;
        ledColors = new Color[LED_COUNT];
        ledStrip = new JPanel() {
            @Override
            protected void paintComponent(Graphics g) {
                super.paintComponent(g);
                for (int i = 0; i < LED_COUNT; i++) {
                    g.setColor(ledColors[i]);
                    g.fillRect(i * (LED_SIZE + LED_SPACING) + LED_SPACING, LED_SPACING, LED_SIZE, LED_SIZE);
                }
            }
        };

        getContentPane().setLayout(new BorderLayout());
        getContentPane().add(ledStrip, BorderLayout.CENTER);

        setVisible(true);
    }

    // time will be pushed from the top level.
    public void setTime(double time) {
        this.time = time;
    }
    public double getCurrentTime() {
        return time;
    }

    public void setPixel(int pixel, int red, int green, int blue) {
        ledColors[pixel] = new Color(red, green, blue);
    }
    public void setPixel(int pixel, Color color) {
        ledColors[pixel] = color;
    }

    public void setPixelWidth(int refpixel, Color color, int width) {
        int pixel;
        for (int i = 0; i < width; i++) {
            pixel = refpixel + i;
            if (pixel >= LED_COUNT) {
                break;
            }
            ledColors[pixel] = color;
        }
    }

    public void setPixel(int pixel, double redp, double greenp, double bluep) {
        // input redp, greenp, bluep in terms of 0.0 .. 1.0 and need to be converted to colors
        int red = (int) (255.0 * redp);
        int green = (int) (255.0 * greenp);
        int blue = (int) (255.0 * bluep);
        ledColors[pixel] = new Color(red, green, blue);
    }
    public void setPixelMix(int pixel, Color c1, Color c2, double ratio) {
        double dred = (c1.getRed() * (1 - ratio)) + (c2.getRed() * ratio);
        double dgreen = (c1.getGreen() * (1 - ratio)) + (c2.getGreen() * ratio);
        double dblue = (c1.getBlue() * (1 - ratio)) + (c2.getBlue() * ratio);
        int red = (int) dred;
        int green = (int) dgreen;
        int blue = (int) dblue;
        ledColors[pixel] = new Color(red, green, blue);
    }


    public int getBufferLength() {
        return LED_COUNT;
    }

}
