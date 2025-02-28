package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ledSubsystem extends SubsystemBase{
    
    private final AddressableLED m_led = new AddressableLED(0); // PWM port 0
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(600); // fix to length

    Timer ledTimerOn;
    Timer ledTimerOff;
    boolean blink;
    int red, green, blue;

    public ledSubsystem(){
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
        ledTimerOff = new Timer();
        ledTimerOn = new Timer();
        ledTimerOn.start();
        ledTimerOff.reset();
        ledTimerOn.reset();
        blink = false;
        red = 0;
        green = 0;
        blue = 0;
    }

    // Set color commands
    public void setRGB(int r, int g, int b){
        for (var i = 0; i < m_ledBuffer.getLength() / 2; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
        red = r;
        green = g;
        blue = b;
        System.out.print(r + g + b);
    }

    public void setColor(Color color) {
        for (var i = 0; i < m_ledBuffer.getLength() / 2; i++) {
            m_ledBuffer.setLED(i, color);
        }
        m_led.setData(m_ledBuffer);
    }
    
    // Blink commands
    public void startBlinking(){
        blink = true;
    }

    public void stopBlinking(){
        blink = false;
    }

    @Override
    public void periodic(){
        if (blink){
            if (ledTimerOn.get() >= .15){
                ledTimerOn.reset();
                ledTimerOn.stop();
                ledTimerOff.start();
                setRGB(red, green, blue);
            }
            else if (ledTimerOff.get() >= .15){
                ledTimerOff.reset();
                ledTimerOff.stop();
                ledTimerOn.start();
                setColor(Color.kBlack);
            }
        }
    }

    // Set to color
    public void setColors(Color color, Color color2) {
        var m_ledCarbo = new Color();
        m_ledCarbo = color;
        for (var i = 0; i < m_ledBuffer.getLength() / 2; i++) {
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
}
