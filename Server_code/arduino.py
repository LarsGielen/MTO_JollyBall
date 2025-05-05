import serial

connection = None

def get_open_connection():
    """
    Returns the current open serial connection.

    :return: The current serial connection if open, None otherwise.
    """
    global connection
    if connection and connection.is_open:
        return connection
    else:
        print("No open serial connection.")
        return None

def open_serial_connection(com_port, baud_rate):
    """
    Opens a serial connection with the specified COM port and baud rate.

    :param com_port: The COM port to connect to (e.g., 'COM3' or '/dev/ttyUSB0').
    :param baud_rate: The baud rate for the serial communication (e.g., 9600).
    :return: A serial.Serial object if successful, None otherwise.
    """
    try:
        ser = serial.Serial(port=com_port, baudrate=baud_rate, timeout=1)
        print(f"Serial connection opened on {com_port} at {baud_rate} baud.")
        global connection
        connection = ser
        return ser
    except serial.SerialException as e:
        print(f"Failed to open serial connection: {e}")
        return None

def close_serial_connection(ser):
    """
    Closes the serial connection.

    :param ser: The serial.Serial object to close.
    """
    if ser and ser.is_open:
        ser.close()
        print("Serial connection closed.")
    else:
        print("Serial connection is already closed or was never opened.")

def send_data(ser, data):
    """
    Sends data to the Arduino over the serial connection.

    :param ser: The serial.Serial object.
    :param data: The data to send (string).
    """
    if ser and ser.is_open:
        try:
            ser.write(data.encode())
        except serial.SerialException as e:
            print(f"Failed to send data: {e}")
    else:
        print("Serial connection is not open.")

def read_data(ser):
    """
    Reads data from the Arduino over the serial connection.

    :param ser: The serial.Serial object.
    :return: The data read from the Arduino (string).
    """
    if ser and ser.is_open:
        try:
            data = ser.readline().decode().strip()
            return data
        except serial.SerialException as e:
            return None
    else:
        print("Serial connection is not open.")
        return None