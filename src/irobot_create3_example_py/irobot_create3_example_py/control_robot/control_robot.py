import rclpy
from rclpy.executors import MultiThreadedExecutor

from irobot_create3_example_py.control_robot import battery_state_subscriber as bss


def main(args=None):
    '''
    Main function - this is the entry point for the ROS application
    '''

    # Initialize the ROS application
    rclpy.init(args=args)

    try:
        # Create a subscriber for the battery state. This is a Node in the ROS graph.
        # Your application can have one node, or multiple
        battery_state = bss.BatteryStateSubscriber()

        # Create an executor - this allows multiple nodes to be run and processing at once.
        # This executor is created with 4 threads, so can easily cope with 4 nodes. For more nodes,
        # add more threads.
        executor = MultiThreadedExecutor(num_threads=4)

        # Add the battery state node to the executor
        executor.add_node(battery_state)

        # Spin the executor. This is a blocking call that starts the executor running allowing
        # messages to be sent and recieved, essentially running a message loop. This call only
        # terminates if something crashes, or if you cancel the run with Ctrl+C
        executor.spin()
    except KeyboardInterrupt:
        # If the user interrupts with Ctrl+C, break out here
        print('Caught keyboard interrupt')
    except BaseException as ex:
        # Anything bad happens, write it out
        print(f'Exception: {ex}')
    finally:
        # Once the application ends, stop the executor so we are no longer processing
        # messages
        executor.shutdown()

        # If we created a battery state, then destroy it to remove it from the ROS graph
        if battery_state:
            battery_state.destroy_node()

        # Stop the ROS application
        rclpy.shutdown()


if __name__ == '__main__':
    main()
