package frc.lib.team3061.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDsRIO extends LEDs {

  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private boolean isGRB;

  protected LEDsRIO() {
    leds = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(ACTUAL_LENGTH);
    // leds.setBitTiming(500, 200, 1200, 1300);
    isGRB = true;

    leds.setLength(ACTUAL_LENGTH);
    leds.setData(buffer);
    leds.start();
  }

  @Override
  protected void updateLEDs() {
    leds.setData(buffer);
  }

  @Override
  protected void setLEDBuffer(int index, Color color) {
    if (isGRB) {
      color = changeToGRB(color);
    }

    buffer.setLED(index, color);
    if (MIRROR_LEDS) {
      buffer.setLED(ACTUAL_LENGTH - index - 1, color);
    }
  }

  static Color changeToGRB(Color color) {
    return new Color(color.green, color.red, color.blue);
  }
}
