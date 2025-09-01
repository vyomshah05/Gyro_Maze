Our system is built around the LilyGO T-Display ESP32 microcontroller board, which serves as the central processing and communication hub. The primary input device is the LSM6DSO 6-degree-of-freedom (accelerometer + gyroscope) sensor, which detects the player’s hand orientation. This data is processed by the TTGO board and used to control two servomotors mounted on perpendicular sides of the maze, tilting the maze along the X and Y axes to move the ball.

To detect when the game is completed, we use a HiLetgo infrared obstacle avoidance sensor placed in the side of the central cup. When the ball drops into the cup, the sensor outputs a signal to the TTGO board, which stops the timer and records the final duration.

For connectivity, the TTGO board uses its built-in Wi-Fi capability to send timing data to Microsoft Azure. The time is logged both in milliseconds and as a formatted minutes/seconds value. Azure serves as the cloud backend for data storage and visualization, enabling scalability for future features like leaderboards or historical performance tracking.

<img width="666" height="450" alt="Screenshot 2025-09-01 at 3 56 19 PM" src="https://github.com/user-attachments/assets/c6b0357c-af6f-4fa4-bd57-0c7eeac37028" />

[Demo video]([url](https://drive.google.com/file/d/1uQYdaZd8vMwwliB8UuHKXI5Kv4btVXlp/view?usp=sharing))
