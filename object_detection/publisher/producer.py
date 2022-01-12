import json
import uuid
from datetime import datetime, timedelta, timezone

from confluent_kafka import SerializingProducer
from confluent_kafka.schema_registry import SchemaRegistryClient
from confluent_kafka.schema_registry.avro import AvroSerializer

from .avro_obj.point3d import Point3d
from .avro_obj.pose3d import Pose3d
from .avro_obj.quaternion import Quaternion
from .config import BOOTSTRAP_SERVERS, SCHEMA_REGISTRY_URL



def get_pose3d_producer(schema_filepath, schema_registry_client):
    key_schema_string = '''
    {
        "logicalType": "timestamp-millis",
        "type": "long"
    }
    '''

    value_schema_string = ''
    with open(schema_filepath) as f:
        value_schema_string = f.read()
    
    key_serialiser = AvroSerializer(schema_registry_client, key_schema_string)
    value_serialiser = AvroSerializer(schema_registry_client, value_schema_string)


    producer_config = {
        "bootstrap.servers": BOOTSTRAP_SERVERS,
        'key.serializer': key_serialiser,
        'value.serializer': value_serialiser
    }

    producer = SerializingProducer(producer_config)
    return producer


def send_pose3d(object_pose: Pose3d, producer: SerializingProducer):
    topic = 'ObjectPose'
    
    key = int((datetime.now(timezone.utc) + timedelta(days=3)).timestamp() * 1e3)
    value = object_pose.avro()
    
    try:
        producer.produce(topic=topic, key=key, value=value, on_delivery=acknowledged)
    except Exception as e:
        print(f"Exception while producing record value - {value} to topic - {topic}: {e}")
    else:
        print(f"Successfully producing record value - {key} and {value} to topic - {topic}")

    producer.flush()

def acknowledged(err, msg):
    if err is not None:
        print("Failed to deliver message: {}".format(err))
    else:
        print("Produced record to topic {} partition [{}] @ offset {}"
                .format(msg.topic(), msg.partition(), msg.offset()))


def get_producer():
    schema_registry_conf = {
        'url': SCHEMA_REGISTRY_URL
    }

    schema_registry_client = SchemaRegistryClient(schema_registry_conf)

    producer = get_pose3d_producer('/home/agvsurvey/agv/agv-survey/object_detection/publisher/schemas/schema-ObjectPose-value-v2.avsc', schema_registry_client)

    return producer

def main():
    
    producer = get_producer()

    while True:
        object_id = int(input('Provide Object ID: '))
        position = map(float, input('Provide position of object in form (x, y, z): ').split(' '))
        orientation = map(float, input('Provide orientation of object in form (x, y, z, w): ').split(' '))

        object_pose = Pose3d(0, object_id, Point3d(*position), Quaternion(*orientation))
        send_pose3d(object_pose, producer)

        input()



if __name__ == '__main__':
    main()
