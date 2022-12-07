# Proyecto-John-Deere
Code made by Emérico Pedraza and Renato Arcos in 2022 for the Linear Actuator Project. </br>

In this project, the main objective is to control a linear actuator from a cloud server using an STM32F401 Black Pill. The signals are intended to be sent via Arduino Cloud to an ESP32 Dev Module, which has to be connected to de Black Pill by a CAN protocol. </br>

Then, the Black Pill uses a PI Control algorythm to move the actuator to the desired position. </br>

