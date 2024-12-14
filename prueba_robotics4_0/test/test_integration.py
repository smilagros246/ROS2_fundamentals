import pytest
import rclpy
import logging
from prueba_robotics4_0.publicador import Publisher  # Replace with your actual Publisher script/module name
from prueba_robotics4_0.suscriptor import Suscriber  # Replace with your actual Subscriber script/module name
from rclpy.executors import SingleThreadedExecutor


# Set up ROS 2 logging configuration
rclpy.init()  # Initialize ROS 2
logger = rclpy.logging.get_logger('test_logger')

# Set logging level for the logger
rclpy.logging.set_logger_level(logger.name, rclpy.logging.LoggingSeverity.INFO)


# Fixture to initialize ROS 2, the nodes, and the executor
@pytest.fixture
def ros2_nodes():
    """
    Fixture to set up and tear down ROS 2 nodes for testing.
    Initializes the ROS 2 communication, creates Publisher and Subscriber nodes, and
    an executor to manage them.
    
    Returns:
        tuple: Publisher node, Subscriber node, and executor.
    """
    talker_node = Publisher()
    listener_node = Suscriber()
    executor = SingleThreadedExecutor()
    executor.add_node(talker_node)
    executor.add_node(listener_node)

    yield talker_node, listener_node, executor

    # Cleanup
    talker_node.destroy_node()
    listener_node.destroy_node()
    rclpy.shutdown()


# Test function for valid message communication
def test_pub_sub(ros2_nodes):
    """
    Tests communication between the Publisher and Subscriber nodes.
    Ensures the Subscriber node receives the messages published by the Publisher node.
    """
    talker_node, listener_node, executor = ros2_nodes
    received_message = None
    timeout = 10  # Timeout in seconds for message reception
    timeout_limit = timeout * 10  # Timeout in iterations (10 cycles per second)

    # Log the start of the test
    logger.info("Starting test for valid message communication.")

    # Run the executor to allow message exchange
    for i in range(timeout_limit):  # Allow for timeout in iterations
        executor.spin_once(timeout_sec=1)
        if listener_node.received_msg:  # Check if the subscriber has received a message
            received_message = listener_node.received_msg
            break

    # Ensure a message was received
    assert received_message is not None, f"Timeout after {timeout} seconds. Subscriber did not receive any messages."

    # Assert the received message format
    assert received_message.startswith("Hello Robotics 4.0:"), (
        f"Unexpected message received: {received_message}. Expected a message starting with 'Hello Robotics 4.0:'."
    )

    # Log the received message
    logger.info(f"Received valid message: {received_message}")
