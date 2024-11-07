# Star Wars Droid Project
This project features an interactive Star Wars-inspired droid, built using Arduino and controlled via a custom smartphone app. The droid is equipped with  a Bluetooth-controlled omniwheel movement, ultrasonic sensors for obstacle detection, and a Bluetooth-controlled buzzer.

This project was created by [Akanksha](https://www.linkedin.com/in/sayheyakanksha/), a Master’s student in the HCI/d program (Class of 2025) at the Luddy School of Informatics, Computing, and Engineering, Indiana University. The droid was built as part of an assignment for the course I590 Prototyping with Arduino Tools, taught by Professor [Matthew Francisco](https://luddy.indiana.edu/contact/profile/?Matthew_Francisco).

## Features
- Omniwheel Movement: Enables the droid to move in all directions with smooth navigation.
- Bluetooth-Controlled App: Custom app for real-time control, including movement command and sound effects.
- Object Detection: Ultrasonic sensor halts the droid when objects are detected within a defined range, ensuring safe interactions.
- Custom Sounds: Play sounds like a "beep-beeep" honk or themed tunes for added personality.

## Components Used
- Arduino Uno (x1) - The core microcontroller.
- DC Motors (x4) - Omniwheel setup for 360-degree movement.
- Servo Motor (x1) - Controls a rotating sensor for scanning.
- Ultrasonic Sensor (x1) - Detects objects in the droid's path.
- Bluetooth Module (x1) - Allows wireless control from the smartphone app.
- Piezo Buzzer (x1) - For sound effects like honks or tunes.

## Note
I am currently facing a conflict with Arduino timers when using the tone() function along with motor control and servo operations. Arduino Uno has limited timers (Timer 0, Timer 1, Timer 2), and tone() uses Timer 2 by default, which is also shared by the motor control functions in the AFMotor library. This caused the motors to stop unexpectedly whenever a tone was played.

I'm still trying to figure out a solution for this. Most probably, I'll end up adding an another arduino nano component and control my sound via that. So, stay tuned! 
