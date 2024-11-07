#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from  rcl_interfaces.msg import Log
import pymongo
import datetime

log_int_to_str = {
    10: 'DEBUG',
    20: 'INFO',
    30: 'WARN',
    40: 'ERROR',
    50: 'FATAL'
}

class MongoLog(Node):
    def __init__(self, level=10, host="localhost", port=27017,
                 database='ros', collection='log', capped=True, size=100000,
                 drop=False):
        super().__init__("log_to_mongodb")
        self.create_subscription(Log, '/rosout', self.callback, 10)
        self.client = pymongo.MongoClient(host, port)
        self.database = self.client[database]

        if collection in self.database.list_collection_names():
            if drop:
                self.database.drop_collection(collection)
                self.collection = self.database.create_collection(
                    collection, capped = capped, size = size)
            else:
                self.collection = self.database[collection]
        else:
            self.collection = self.database.create_collection(
                collection, capped = capped, size = size)

        self.get_logger().info('Logging everything to mongodb: started')

    def callback(self, msg):
        self.collection.insert_one({
            'when': datetime.datetime.fromtimestamp(msg.stamp.sec),
            'levelno': msg.level,
            'levelname': log_int_to_str[msg.level],
            'message': msg.msg,
            'file': msg.file,
            'function': msg.function,
            'line': msg.line
        })

def main():
    rclpy.init()
    node = MongoLog()
    rclpy.spin(node)
