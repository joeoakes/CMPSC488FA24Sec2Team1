import logging
import pymongo
import datetime
from bson.codec_options import CodecOptions

DB_CONN = "mongodb://127.0.0.1:27017/"
client = pymongo.MongoClient(DB_CONN)

class MongoLogHandler(logging.Handler):
    def __init__(self, level=logging.NOTSET, host="localhost", port=27017,
                 database='db', collection='log', capped=True, size=100000,
                 drop=False):
        logging.Handler.__init__(self, level)
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

    def emit(self, record):
        self.collection.insert_one({'when': datetime.datetime.now(),
                              'levelno': record.levelno,
                              'levelname': record.levelname,
                              'message': record.msg})


if __name__ == '__main__':
    mgl = MongoLogHandler(database="logDB")
    l = logging.getLogger("test")
    l.setLevel(logging.INFO)
    l.addHandler(mgl)
    l.info("Testing 123")
