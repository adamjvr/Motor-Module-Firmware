# Motor-Module-Firmware

Close loop control firmware for DC motor drive modules for industrial robots. Supports different control input methods, including USB, SPI, UART, and digital control.

## Functionality

The sketch allows you to control a DC motor with closed-loop PID using a quadrature encoder. The motor can be controlled in terms of speed and direction based on the chosen control input method.

### Control Input Methods

1. **USB Control (Serial Communication)**
   - Connect the Arduino to your computer via USB.
   - Upload the sketch to the Arduino using the Arduino IDE.
   - Open the Serial Monitor to send commands ('F' for forward, 'B' for backward, 'S' for stop).

2. **SPI Control**
   - Connect the Arduino to your SPI master device.
   - Upload the sketch to the Arduino using the Arduino IDE.
   - Ensure proper SPI communication settings with your master device.

3. **UART Control**
   - Connect the Arduino to your UART-enabled device.
   - Upload the sketch to the Arduino using the Arduino IDE.
   - Send commands ('F' for forward, 'B' for backward, 'S' for stop) using a UART-enabled device.

4. **Digital Control**
   - Connect the digital control signals for direction and braking, as well as the PWM signal for speed control.
   - Upload the sketch to the Arduino using the Arduino IDE.

## Compilation Instructions

### USB Control
1. Connect the Arduino to your computer via USB.
2. Open the sketch in the Arduino IDE.
3. Choose the correct board and port in the Tools menu.
4. Click the "Upload" button to compile and upload the sketch to the Arduino.

### SPI Control
1. Connect the Arduino to your SPI master device.
2. Open the sketch in the Arduino IDE.
3. Choose the correct board and port in the Tools menu.
4. Click the "Upload" button to compile and upload the sketch to the Arduino.

### UART Control
1. Connect the Arduino to your UART-enabled device.
2. Open the sketch in the Arduino IDE.
3. Choose the correct board and port in the Tools menu.
4. Click the "Upload" button to compile and upload the sketch to the Arduino.

### Digital Control
1. Connect the digital control signals for direction and braking, as well as the PWM signal for speed control.
2. Open the sketch in the Arduino IDE.
3. Choose the correct board and port in the Tools menu.
4. Click the "Upload" button to compile and upload the sketch to the Arduino.

## Notes
- Adjust motor control pins, PID parameters, and other configurations based on your specific hardware setup.
- Ensure proper wiring and connections for your motor, encoder, and control signals.

## Contributing

Feel free to contribute to the development of this library. If you find any issues or have suggestions for improvements, please create an issue or submit a pull request.

## License

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.