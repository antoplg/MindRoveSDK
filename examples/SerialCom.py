import serial


class SerialCommunication:
    """
    Class for serial communication with Arduino
    """

    def __init__(self, com_port, baud_rate):
        self.ser = serial.Serial(com_port, baud_rate)

    def send_command_to_arduino(self, command):
        self.ser.reset_output_buffer()
        self.ser.write(command.encode())

    def receive_data_from_arduino(self):
        self.ser.reset_input_buffer()
        message = ""
        while True:
            # Get data from TCP server
            data = self.ser.readline()
            data = data.decode("utf-8")

            message = message + data

            # Search for unique first element of message "<" (find returns -1 if element is not found)
            position_first_element = message.find("<")

            # Check if the unique first element is included in the data and cut everything before if so
            if position_first_element == -1:
                continue
            else:
                message = message[position_first_element:]

            # Search for the unique last element of message ">"
            position_last_element = message.find(">")

            # Check if the unique last element is included in the data and cut everything after if so
            if position_last_element == -1:
                continue
            else:
                message = message[:position_last_element+1]
                break

        return message