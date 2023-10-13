import threading
import queue
import time
from main import *
    # main(robot_type=RobotType.rectangle)rectangle
# time.sleep(10)
producer_thread = threading.Thread(target=producer, args=(data_queue,data_queue2))
consumer_thread = threading.Thread(target=consumer, args=(data_queue,data_queue2))
producer_thread.start()
consumer_thread.start()

producer_thread.join()

consumer_thread.join()