# Appendix A: Hardware BOM

## A.1 Introduction: Building Your Robot Platform

A well-defined Hardware Bill of Materials (BOM) is crucial for any robotics project. It provides a clear list of components, their specifications, and estimated costs, guiding the procurement and assembly process. This appendix details a comprehensive BOM for building a robust Jetson-based mobile robot platform suitable for research, education, and prototyping. The focus is on providing a balanced approach to performance and cost, offering a practical foundation for exploring physical AI concepts.

## A.2 Core Processing Unit: NVIDIA Jetson Orin NX

The NVIDIA Jetson Orin NX is selected as the primary computational platform due to its unparalleled AI performance per watt, small form factor, and robust developer ecosystem. It provides the necessary horsepower for running complex perception pipelines and advanced control algorithms at the edge.

### Component Details:
*   **Item:** NVIDIA Jetson Orin NX Developer Kit
*   **Description:** System-on-Module with NVIDIA Ampere architecture GPU, multiple ARM Cortex-A78AE CPUs, deep learning accelerators. Ideal for edge AI.
*   **Estimated Unit Cost:** $499 - $599 (varies by retailer/configuration)
*   **Vendor:** NVIDIA, authorized distributors
*   **Link:** [QR Code: Link to NVIDIA Jetson Orin NX Developer Kit page]

## A.3 Sensor Suite: The Robot's Eyes and Ears

A diverse set of sensors is essential for the robot to perceive its environment, localize itself, and interact with objects.

### Depth Camera: Intel RealSense D435i
*   **Item:** Intel RealSense Depth Camera D435i
*   **Description:** Stereo-based depth camera with an integrated IMU, providing high-quality depth and RGB streams.
*   **Estimated Unit Cost:** $199 - $249
*   **Vendor:** Intel, Amazon, authorized distributors
*   **Link:** [QR Code: Link to Intel RealSense D435i product page]

### 2D LiDAR: Slamtec RPLIDAR A3
*   **Item:** Slamtec RPLIDAR A3
*   **Description:** High-performance 360-degree laser range scanner, ideal for 2D mapping and navigation.
*   **Estimated Unit Cost:** $399 - $499
*   **Vendor:** Slamtec, AliExpress, authorized distributors
*   **Link:** [QR Code: Link to RPLIDAR A3 product page]

### IMU (External): Adafruit BNO055 (or similar)
*   **Item:** Adafruit BNO055 Absolute Orientation Sensor
*   **Description:** 9-DOF sensor providing orientation, acceleration, and angular velocity data, useful for robust state estimation.
*   **Estimated Unit Cost:** $30 - $50
*   **Vendor:** Adafruit, SparkFun, local electronics stores
*   **Link:** [QR Code: Link to Adafruit BNO055 product page]

## A.4 Mobility Platform Components

These components form the basis of the robot's physical movement. For this example, we'll consider a simple wheeled mobile base.

### Microcontrollers/Motor Drivers: ESP32-based Controller + L298N
*   **Item:** ESP32 Development Board (e.g., ESP32-WROOM-32)
*   **Description:** Low-cost, Wi-Fi/Bluetooth enabled microcontroller for motor control and interfacing.
*   **Estimated Unit Cost:** $10 - $20
*   **Vendor:** Espressif, Amazon, AliExpress
*   **Item:** L298N Motor Driver Module
*   **Description:** Dual H-bridge motor driver for controlling DC motors.
*   **Estimated Unit Cost:** $5 - $10
*   **Vendor:** Amazon, AliExpress, local electronics stores

### Motors and Wheels: DC Geared Motors with Encoders
*   **Item:** 2x DC Geared Motors with Encoders (e.g., JGB37-520)
*   **Description:** Provides propulsion and odometry feedback.
*   **Estimated Unit Cost:** $20 - $30 each
*   **Vendor:** AliExpress, Pololu

### Battery and Power Management: LiPo Battery + UBEC/Buck Converter
*   **Item:** 11.1V 5200mAh 3S LiPo Battery
*   **Description:** Power source for the robot.
*   **Estimated Unit Cost:** $40 - $70
*   **Vendor:** HobbyKing, local RC stores
*   **Item:** UBEC / DC-DC Buck Converter (e.g., LM2596-based)
*   **Description:** Regulates voltage for Jetson (12V) and microcontroller (5V).
*   **Estimated Unit Cost:** $5 - $15
*   **Vendor:** Amazon, AliExpress

## A.5 Miscellaneous Components and Enclosure

*   **Wiring:** Assorted jumper wires, power cables (18-22 AWG).
*   **Fasteners:** M3/M4 screws, nuts, standoffs.
*   **Enclosure/Frame:** Acrylic, aluminum profiles, or 3D printed parts for chassis.

## A.6 Comprehensive Bill of Materials Table

| Item No. | Component                          | Description                                                                     | Quantity | Est. Unit Cost | Est. Total Cost | Vendor         |
| :------- | :--------------------------------- | :------------------------------------------------------------------------------ | :------- | :------------- | :-------------- | :------------- |
| 1        | NVIDIA Jetson Orin NX Dev Kit    | Core compute for AI and robotics                                                | 1        | $550           | $550            | NVIDIA         |
| 2        | Intel RealSense D435i              | Depth camera with IMU for 3D perception                                         | 1        | $220           | $220            | Intel          |
| 3        | Slamtec RPLIDAR A3                 | 360-degree 2D LiDAR for mapping & navigation                                    | 1        | $450           | $450            | Slamtec        |
| 4        | Adafruit BNO055 IMU                | 9-DOF absolute orientation sensor                                               | 1        | $40            | $40             | Adafruit       |
| 5        | ESP32 Dev Board                    | Microcontroller for motor control                                               | 1        | $15            | $15             | Espressif      |
| 6        | L298N Motor Driver Module          | Drives 2x DC motors                                                             | 1        | $8             | $8              | Generic        |
| 7        | DC Geared Motor with Encoder       | 12V DC motor for drive system (e.g., JGB37-520)                                 | 2        | $25            | $50             | Generic        |
| 8        | 11.1V 5200mAh 3S LiPo Battery      | Main power source                                                               | 1        | $55            | $55             | HobbyKing      |
| 9        | UBEC / Buck Converter              | Power regulation for Jetson & peripherals                                       | 1        | $10            | $10             | Generic        |
| 10       | Wiring, Fasteners, Enclosure       | Assorted wires, screws, nuts, and basic chassis materials                       | 1        | $100           | $100            | Various        |
|          | **TOTAL ESTIMATED COST**         |                                                                                 |          |                | **$1518**       |                |

*Note: Prices are approximate and subject to change based on vendor, time of purchase, and region.*

## A.7 Wiring Diagram (Conceptual)

**Diagram Placeholder: Simplified Wiring Diagram for Jetson-based Mobile Robot**
*(A diagram illustrating the connections between the Jetson Orin NX, LiPo battery, UBEC, ESP32 microcontroller, L298N motor driver, DC motors, LiDAR, depth camera, and IMU. Focus on power and data lines.)*

## A.8 Conclusion

This detailed BOM serves as a foundational guide for constructing a functional physical AI platform. By carefully sourcing and assembling these components, you will have a capable robot ready to explore advanced concepts in perception, control, and autonomous navigation discussed throughout this book. The flexible nature of these components also allows for customization and upgrades as your project evolves.
