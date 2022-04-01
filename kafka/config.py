from inspect import getsourcefile  
from pathlib import Path

cwd = Path(getsourcefile(lambda:0)).parent


BOOTSTRAP_SERVERS = 'localhost:9092'
SCHEMA_REGISTRY_URL = 'http://localhost:8081'

OBJECTPOSE_TOPIC = 'agv_survey.object_pose'
OBJECTPOSE_SCHEMA_FILE = Path(cwd, 'schemas', 'schema-ObjectPose-value-v3.avsc')
MIRPOSE_TOPIC = 'mirposestamped.mir100-agv'
MIRPOSE_SCHEMA_FILE = Path(cwd, 'schemas', 'MiRPoseStamped.avsc')