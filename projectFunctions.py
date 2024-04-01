import pandas as pd
from pyfirmata import Arduino, util, pyfirmata
import time
import serial


def displayData():
    board = Arduino("COM3")  # Initialize Arduino board
    button_pin1 = 2
    button_pin2 = 4
    position = 0
    personal = ["knee", "hip", "shoulder", "elbow"]
    optimal_angles = [108.5, 122.3, 123.4, 71.9]
    df = pd.read_csv("personal.csv", usecols=personal)

    # Start iterator thread so that serial buffer doesn't overflow
    it = util.Iterator(board)
    it.start()

    # Define button as INPUT
    board.digital[button_pin1].mode = pyfirmata.INPUT
    board.digital[button_pin2].mode = pyfirmata.INPUT

    while True:
        # Read button state
        button1 = board.digital[button_pin1].read()
        button2 = board.digital[button_pin2].read()
        time.sleep(0.1)

        if button1 == 1 and button2 == 1:
            break

        elif button1 == 1 and button2 == 0:
            position -= 1
            if position < 0:
                position = 0
            send(f"your {personal[position]} was {sum(df.personal[position]) / len(df.personal[position]) - optimal_angles[position]} deg off")
            time.sleep(0.5)

        elif button1 == 0 and button2 == 1:
            position += 1
            if position > 3:
                position: int = 3
            send(
                f"your {personal[position]} was {sum(df.personal[position]) / len(df.personal[position]) - optimal_angles[position]} deg off")
            time.sleep(0.5)


def send(message):
    ser = serial.Serial('COM3', 9600)  # Adjust the port and baud rate as needed

    # Send message to Arduino
    ser.write(message.encode())

    # Close serial connection
    ser.close()

