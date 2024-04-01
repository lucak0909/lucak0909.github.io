from pyfirmata import Arduino, util, pyfirmata
import time

# Define the digital pin connected to the button
button_pin1 = 2
button_pin2 = 4
num_B1 = 0
num_B2 = 0
bothPressed = False

# Initialize Arduino board
board = Arduino("COM3")

# Start iterator thread so that serial buffer doesn't overflow
it = util.Iterator(board)
it.start()

# Define button as INPUT
board.digital[button_pin1].mode = pyfirmata.INPUT
board.digital[button_pin2].mode = pyfirmata.INPUT


def waitForPress():
    while True:
        # Read button state
        button1 = board.digital[button_pin1].read()
        button2 = board.digital[button_pin2].read()
        time.sleep(0.1)
        if button1 == 1 and button2 == 1:
            break
    return True


# below is for testing
for i in range(100):
    # Read button state
    button1_state = board.digital[button_pin1].read()
    button2_state = board.digital[button_pin2].read()
    print(f"\n1 : {button1_state}"
          f"\n2 : {button2_state}\n")

    if button1_state:
        num_B1 += 1
    if button2_state:
        num_B2 += 1

    # Check if button is pressed (assuming LOW when pressed)
    if button1_state == 1 and button2_state == 1:
        print("Both Pressed")
        # Do something when the button is pressed

    time.sleep(0.1)

print(f"1 was pressed {num_B1} times \n2 was pressed {num_B2} times")
