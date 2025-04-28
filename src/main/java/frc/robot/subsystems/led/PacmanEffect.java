package frc.robot.subsystems.led;
import edu.wpi.first.wpilibj.util.Color;

public class PacmanEffect {
    // Pacman game variables
    private int pacWidth = 3; // pixel width of each object, must be larger than 1
    private double pacmanPosition;
    private double[] ghostPositions;
    private int pelletPosition = 13; // Pellet near the left
    private boolean pelletEaten = false;
    private boolean pacmanEating = false;
    private double pacmanNormalSpeed = 0.5; // Pixels per second
    private double pacmanEatingSpeed = 1.0;
    private double ghostSpeed = 0.4;
    private int pacmanDirection = -1; // 1 = right, -1 = left

    private Color[] ghostColors = {Color.kRed, Color.kPink, Color.kCyan, Color.kOrange};
    private Color pacmanColor = Color.kYellow;
    private Color ghostFleeColor = Color.kBlue;
    private Color ghostFleeColor2 = Color.kWhite;
    private int numLeds;

    public PacmanEffect(int numLeds) {
        this.numLeds = numLeds;
        ghostPositions = new double[4];
        resetPositions();
    }

    private void resetPositions() {
        pelletEaten = false;
        pacmanEating = false;
        pacmanDirection = -1;
        pacmanPosition = numLeds - 1;
        ghostPositions[0] = numLeds + 10;
        ghostPositions[1] = numLeds + 18;
        ghostPositions[2] = numLeds + 26;
        ghostPositions[3] = numLeds + 34;
    }

    public void update(LedStrip ledStrip) {
        updateGame(ledStrip);
        updateDisplay(ledStrip);
    }

    private void updateGame(LedStrip ledStrip) {
        double pacmanSpeed;
        // Update Pacman position
        if (pelletEaten) {
            pacmanDirection = 1; // ate the pellet, now go right
            pacmanSpeed = pacmanEatingSpeed;
            pacmanEating = true;

        } else {
            pacmanDirection = -1 ; // go left
            pacmanSpeed = pacmanNormalSpeed;
            pacmanEating = false;
        }

        pacmanPosition += pacmanSpeed * pacmanDirection;

        // Limit pacman to the strip length.
        if (pacmanPosition < 0 || pacmanPosition >= numLeds) {
            resetPositions();
        }

        // Update Ghost positions
        for (int i = 0; i < ghostPositions.length; i++) {
            // if a ghost touches pacman then put that ghost off the screen
            if (Math.abs(pacmanPosition - ghostPositions[i]) < 2) {
                ghostPositions[i] = numLeds * 2; 
            } else { // move ghost normally
                ghostPositions[i] = ghostPositions[i] + pacmanDirection * ghostSpeed;
            }
        }

        // Check if Pacman has reached the pellet
        if (!pelletEaten && Math.abs(pacmanPosition - pelletPosition) <= 2) {
            pelletEaten = true;
        }
    }

    private void updateDisplay(LedStrip ledStrip) {

        int flashtime = (int) (ledStrip.getCurrentTime() / 0.2);

        // Clear the strip
        for (int i = 0; i < numLeds; i++) {
            ledStrip.setPixel(i, Color.kBlack);
        }

        // Draw pellet
        if (!pelletEaten) {
            if ((int) (ledStrip.getCurrentTime() * 2) % 2 == 0) { // Flash every 0.5 seconds
                ledStrip.setPixelWidth(pelletPosition, Color.kWhite, pacWidth);
            }
        }

        // Draw Pacman.  Ensure pacmanPosition is within bounds or else dont draw him.
        if (pacmanPosition >= 0 && pacmanPosition < numLeds) {
            int mouthinset = (flashtime % 2)*2;
            int pacpos = (int) pacmanPosition;
            if (pacmanDirection > 0) { // right
                ledStrip.setPixelWidth(pacpos, pacmanColor, pacWidth-mouthinset);
            } else {
                ledStrip.setPixelWidth(pacpos + mouthinset, pacmanColor, pacWidth-mouthinset);
            }
        }

        // Draw Ghosts
        for (int i = 0 ; i < ghostPositions.length ; i++) {
            Color ghostColor;
            if (pacmanEating) {
                // flash between blue and white
                ghostColor = flashtime % 2 == 0 ? ghostFleeColor : ghostFleeColor2;
            } else {
                // not eating, normal colors
                ghostColor = ghostColors[i];
            }
            // Ensure ghostPositions[i] is within bounds before drawing
            if (ghostPositions[i] >= 0 && ghostPositions[i] < numLeds) {
                ledStrip.setPixelWidth((int) ghostPositions[i], ghostColor, pacWidth);
            }
        }

    }

}
