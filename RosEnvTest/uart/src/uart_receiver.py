
import rospy
import serial
from your_package_name.msg import MergedCoordinates

def uart_receiver():
    # Initialize the ROS node
    rospy.init_node('uart_receiver', anonymous=True)

    # Define the UART ports and baud rates
    uart_ports = ['/dev/ttyAMA0', '/dev/ttyAMA1']  # Replace with your UART ports
    baud_rates = [9600, 9600]  # Replace with your baud rates

    # Create a list to hold the UART objects
    uart_objects = []

    # Open the UART ports
    for port, baud_rate in zip(uart_ports, baud_rates):
        uart = serial.Serial(port, baud_rate, timeout=0.1)
        uart_objects.append(uart)

    # Create a ROS publisher for the merged coordinates
    pub = rospy.Publisher('merged_coordinates', MergedCoordinates, queue_size=10)

    # Start reading data from the UARTs
    while not rospy.is_shutdown():
        # Create a new merged coordinates message
        merged_coordinates = MergedCoordinates()

        for uart in uart_objects:
            # Read the UART data
            data = uart.readline().decode().strip()

            # Process the received data based on the UART port
            if uart.port == '/dev/ttyAMA0':
                try:
                    merged_coordinates.x = float(data)
                except ValueError:
                    rospy.logwarn("Invalid data received for x coordinate: %s", data)
            elif uart.port == '/dev/ttyAMA1':
                try:
                    merged_coordinates.y = float(data)
                except ValueError:
                    rospy.logwarn("Invalid data received for y coordinate: %s", data)

        # Publish the merged coordinates message
        pub.publish(merged_coordinates)

    # Close the UART ports
    for uart in uart_objects:
        uart.close()

if __name__ == '__main__':
    try:
        uart_receiver()
    except rospy.ROSInterruptException:
        pass