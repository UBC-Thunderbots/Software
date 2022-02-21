import shared.parameter.python_bindings as py
from proto.sensor_msg_pb2 import SensorProto

def main():
    sensor_proto = SensorProto();
    sensor_fusion_config = py.SensorFusionConfig()
    sensor_fusion = py.SensorFusion(sensor_fusion_config)
    sensor_fusion.processSensorProto(sensor_proto)
    world = sensor_fusion.getWorld()
    print(world)

if __name__ == "__main__":
    main()
