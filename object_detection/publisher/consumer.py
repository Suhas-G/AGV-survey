from confluent_kafka import DeserializingConsumer
from confluent_kafka.schema_registry import SchemaRegistryClient
from confluent_kafka.schema_registry.avro import AvroDeserializer
from config import BOOTSTRAP_SERVERS, OBJECTPOSE_TOPIC, SCHEMA_REGISTRY_URL


def get_pose3d_consumer(schema_filepath, schema_registry_client):
    key_schema_string = '''
    {
        "logicalType": "timestamp-millis",
        "type": "long"
    }
    '''

    value_schema_string = ''
    with open(schema_filepath) as f:
        value_schema_string = f.read()
    
    key_deserialiser = AvroDeserializer(schema_registry_client, key_schema_string)
    value_deserialiser = AvroDeserializer(schema_registry_client, value_schema_string)

    consumer_config = {
        "bootstrap.servers": BOOTSTRAP_SERVERS,
        'group.id': 'local-consumer',
        'key.deserializer': key_deserialiser,
        'value.deserializer': value_deserialiser
    }
    consumer = DeserializingConsumer(consumer_config)

    consumer.subscribe([OBJECTPOSE_TOPIC])

    return consumer

def main():
    schema_registry_conf = {
        'url': SCHEMA_REGISTRY_URL
    }

    schema_registry_client = SchemaRegistryClient(schema_registry_conf)
    consumer = get_pose3d_consumer('/home/agvsurvey/agv/agv-survey/object_detection/publisher/schemas/schema-ObjectPose-value-v2.avsc', schema_registry_client)

    while True:
        try:
            msg = consumer.poll(1.0)
            if msg is None:
                # No message available within timeout.
                # Initial message consumption may take up to
                # `session.timeout.ms` for the consumer group to
                # rebalance and start consuming
                print("Waiting for message or event/error in poll()")
                continue
            elif msg.error():
                print('error: {}'.format(msg.error()))
            else:
                key = msg.key()
                value = msg.value()
                print(f'Key: {key}\nValue: {value}')
        except KeyboardInterrupt:
            break
        except Exception as e:
            # Report malformed record, discard results, continue polling
            print("Message deserialization failed {}".format(e))
            pass

if __name__ == '__main__':
    main()