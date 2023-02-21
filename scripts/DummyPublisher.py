import logging


class DummyPublisher:
    def __init__(self, topic:str):
        logging.basicConfig(level=logging.INFO)
        self.topic = topic

    def publish(self, toBePublishedData: str):
        logging.info(f"DummyPublisher sendet Message \"{self.topic} {toBePublishedData}\"")
