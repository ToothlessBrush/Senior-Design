# Flight Controller


## Flight Controller pinout
with programming pins on top
| Connection | Left Side  | Right Side | Connection |
| -------- | ---------- | ---------- | ------- |
|          | VB         | 3v3        |         |
|          | C13        | G          |         |
|          | C14        | 5V         |         |
|          | C15        | B9         |         |
|          | R          | B8         |         |
| IMU INT1 | A0 (EXTI0) | B7 (I2C)   |         |
|          | A1         | B6 (I2C)   |         |
| LORA RX  | A2 (TX)    | B5         | Motor 2 |
| LORA TX  | A3 (RX)    | B4         | Motor 1 |
| IMU CS   | A4 (GPIO)  | B3         |         |
| IMU SPC  | A5 (SCK)   | A15        |         |
| IMU SDO  | A6 (MISO)  | A12        | Uart6 rx ( optical ) |
| IMU SDI  | A7 (MOSI   | A11        | Uart6 tx ( optical ) |
| Motor 3  | B0         | A10        | uart1 rx ( elrs ) |
| Motor 4  | B1         | A9         | Uart1 tx ( elrs ) |
| SONAR echo | B2         | A8         |         |
| SONAR trig | B10        | B15        | SPI2_MOSI ( motor control ) |
|          | 3v3        | B14        | SPI2_MISO ( motor control ) |
|          | G          | B13        | SPI2_SK ( motor control ) |
|          | 5V         | B12        |  SPI2_CS?       |
