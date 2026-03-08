"""
Team 1504 - LEDSubsystem
Communicates robot state to drive team via addressable LEDs.

Inspired by 6328's "VirtualSubsystem" pattern - no command requirements,
just runs alongside everything else every loop.

States (priority order - highest first):
  E_STOP       → flashing red
  HAS_FUEL     → solid green
  AT_SPEED     → blinking white (shooter ready)
  CLIMBING     → solid purple
  DEFAULT      → team colors (blue/white)
"""

import commands2
import wpilib

from src.constants import LEDConstants


class LEDSubsystem(commands2.Subsystem):
    """
    LED subsystem that is never bound to commands (no requirements set).
    Call update_state() from RobotContainer or other subsystems each loop.
    """

    def __init__(self) -> None:
        super().__init__()
        self._leds = wpilib.AddressableLED(LEDConstants.kLEDPort)
        self._buffer = [wpilib.AddressableLED.LEDData() for _ in range(LEDConstants.kLEDLength)]
        self._leds.setLength(LEDConstants.kLEDLength)
        self._leds.setData(self._buffer)
        self._leds.start()

        self._blink_timer = wpilib.Timer()
        self._blink_timer.start()
        self._blink_on = True

        # State flags (set externally each loop)
        self.has_fuel    = False
        self.at_speed    = False
        self.is_climbing = False
        self.is_estopped = False

    def periodic(self) -> None:
        # Blink toggle at 4 Hz
        if self._blink_timer.advanceIfElapsed(0.125):
            self._blink_on = not self._blink_on

        if self.is_estopped:
            self._set_all(255, 0, 0) if self._blink_on else self._set_all(0, 0, 0)
        elif self.at_speed:
            self._set_all(255, 255, 255) if self._blink_on else self._set_all(0, 0, 0)
        elif self.has_fuel:
            self._set_all(0, 200, 0)
        elif self.is_climbing:
            self._set_all(128, 0, 200)
        else:
            # Team colors: alternating blue/white
            for i in range(LEDConstants.kLEDLength):
                if i % 4 < 2:
                    self._buffer[i].setRGB(0, 0, 200)
                else:
                    self._buffer[i].setRGB(200, 200, 200)

        self._leds.setData(self._buffer)

    def _set_all(self, r: int, g: int, b: int) -> None:
        for led in self._buffer:
            led.setRGB(r, g, b)
