# Switched Mode Power Supply Programmable Controller 
---

By definition, a switch mode power supply (SMPS) is a type of power supply that uses semiconductor switching techniques, rather than standard linear methods to provide the required output voltage. The basic switching converter consists of a power switching stage and a control circuit.

(SMPS) is an electronic circuit that converts power using switching devices that are turned on and off at high frequencies, and storage components such as inductors or capacitors to supply power when the switching device is in its non-conduction state. Switching power supplies have high efficiency and are widely used in a variety of electronic equipment, including computers and other sensitive equipment requiring stable and efficient power supply.

A switched-mode power supply is also known as a switch-mode power supply or switching-mode power supply.

Linear voltage IC regulators have been the basis of power supply designs for many years as they are very good at supplying a continuous fixed voltage output.

![smps](https://res.cloudinary.com/rsc/image/upload/b_rgb:FFFFFF,c_pad,dpr_1.0,f_auto,h_843,q_auto,w_1500/c_pad,h_843,w_1500/F2317776-01?pgw=1&pgwact=1)

Linear voltage regulators are generally much more efficient and easier to use than equivalent voltage regulator circuits made from discrete components such a zener diode and a resistor, or transistors and even op-amps.

The most popular linear and fixed output voltage regulator types are by far the 78… positive output voltage series, and the 79… negative output voltage series. These two types of complementary voltage regulators produce a precise and stable voltage output ranging from about 5 volts up to about 24 volts for use in many electronic circuits.

There is a wide range of these three-terminal fixed voltage regulators available each with its own built-in voltage regulation and current limiting circuits. This allows us to create a whole host of different power supply rails and outputs, either single or dual supply, suitable for most electronic circuits and applications.

How a Switched-Mode Power Supply Works
A basic AC-DC SMPS consists of:

1. Input rectifier and filter
2. Transformer 
3. Output rectifier and filter
4. Feedback and control circuit

![smps](https://www.altronix.com/images/products/smp7pm/main.jpg)

In order to use an Arduino in this task, we can use PWM outputs of the board to control the output voltage. Pulse width modulation (PWM) is a common technique used to vary the width of pulses in a train of pulses.

PWM pins are used in many applications, including dimming an LED, providing variable speed control for motors, and more. Most importantly they are used to provide an analog output, providing an analog voltage between 0% and 100% if the digital output is filtered. On the Arduino UNO board, digital pins 3, 5, 6, 9, 10, and 11 are PWM pins.

Required Hardware
- Transformer and Rectifier Circuit
- Transformer (Primary to Secondary turns ratio 4.107)
- 4 x MBR20100 Schottky diodes
- 2 x 1F Capacitors
- 2 x 100kΩ resistors

![smps](https://www.maximintegrated.com/content/dam/images/design/videos/2021/vid-recommended-troubleshooting-techniques-for-buck-smps-circuits.jpg)