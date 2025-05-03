from MSCL import mscl
import time

# Device paremeters
COM_PORT = "COM20"
GPIO_PIN = 1

# Setup connection
try:
    #create a Serial Connection with the specified COM Port, default baud rate of 921600
    connection = mscl.Connection.Serial(COM_PORT)

    #create an InertialNode with the connection
    node = mscl.InertialNode(connection)
except mscl.Error as e:
    print("Error:", e)
    exit(1)

# To use an GPIO pin, we first need to configure it
try:
    new_config = mscl.GpioConfiguration()
    new_config.pin = GPIO_PIN  # GPIO pin number
    new_config.feature = new_config.GPIO_FEATURE  # Set pin as GPIO
    new_config.behavior = new_config.GPIO_INPUT  # Set GPIO pin as input
    node.setGpioConfig(new_config)  # Apply the configuration
except mscl.Error as e:
    print("Error:", e)
    exit(1)

# Simple while loop to monitor the GPIO state
prev_time = time.time()
while True:
    try:
        sw = node.getGpioState(1)
        print("GPIO State: ", sw)
        time.sleep(0.001)

        # # Calculate the instantaneous loop rate
        # now = time.time()
        # dt = now - prev_time
        # prev_time = now
        # if dt > 0:
        #     rate = 1 / dt
        #     print(f"Instantaneous loop rate: {rate:.2f} Hz")

    except mscl.Error as e:
        print("Error:", e)
        break
