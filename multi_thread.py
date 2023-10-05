import threading
import queue
from main import *
    # main(robot_type=RobotType.rectangle)rectangle
producer_thread = threading.Thread(target=producer, args=(data_queue,))
consumer_thread = threading.Thread(target=consumer, args=(data_queue,))
producer_thread.start()
consumer_thread.start()

producer_thread.join()

consumer_thread.join()