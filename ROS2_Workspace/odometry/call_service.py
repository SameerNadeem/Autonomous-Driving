# call_service.py
import rclpy
from std_srvs.srv import Empty

def call_reset_odometry_service(node):
    # Create a service client for the 'reset_odometry' service
    client = node.create_client(Empty, 'reset_odometry')

    # Wait for the service to be available
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')

    # Create a request of type Empty (no data needed in request)
    request = Empty.Request()

    # Call the service asynchronously
    future = client.call_async(request)

    # Handle the result (no response expected, just check if the service call was successful)
    future.add_done_callback(lambda future: service_callback(future, node))

def service_callback(future, node):
    try:
        # Since there's no response data, just check if the call was successful
        future.result()  # If no exception is raised, the service call was successful
        node.get_logger().info('Service call successful!')
    except Exception as e:
        node.get_logger().error(f'Service call failed: {e}')
