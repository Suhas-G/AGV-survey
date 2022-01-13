import json
import uuid
from datetime import datetime, timedelta, timezone


from confluent_kafka import SerializingProducer
from confluent_kafka.schema_registry import SchemaRegistryClient
from confluent_kafka.schema_registry.avro import AvroSerializer


from .avro_obj.ObjectPose3D import ObjectPose3D
from .config import BOOTSTRAP_SERVERS, SCHEMA_REGISTRY_URL, SCHEMA_FILE, OBJECTPOSE_TOPIC



def get_pose3d_producer(schema_filepath, schema_registry_client):
    key_schema_string = '''
    {
        "type": "record",
        "name": "SimpleKey",
        "namespace": "de.dfki.cos.mrk40.avro",
        "fields": [
            {
                "name": "key",
                "type": "string"
            }
        ]
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


def send_pose3d(object_pose: ObjectPose3D, producer: SerializingProducer):
    topic = OBJECTPOSE_TOPIC
    
    key = {'key': 'myKey'} # str(uuid.uuid4())
    value = object_pose.dict()
    
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

    producer = get_pose3d_producer(SCHEMA_FILE, schema_registry_client)

    return producer

