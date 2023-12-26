from proto.message_translation import tbots_protobuf
import software.python_bindings as tbots_cpp
from software.thunderscope.constants import ProtoUnixIO
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

import numpy

def async_sim_ticker(blue_proto_unix_io: ProtoUnixIO, yellow_proto_unix_io: ProtoUnixIO, sim_proto_unix_io: ProtoUnixIO):
    blue_primitive_set_buffer = ThreadSafeBuffer(buffer_size=1, protobuf_type=PrimitiveSet)
    yellow_primitive_set_buffer = ThreadSafeBuffer(buffer_size=1, protobuf_type=PrimitiveSet)

    blue_proto_unix_io.register_observer(PrimitiveSet, blue_primitive_set_buffer)
    yellow_proto_unix_io.register_observer(PrimitiveSet, yellow_primitive_set_bufer)




def sync_simulation(sim_proto_unix_io: ProtoUnixIO, num_robots: int):
   world_state_received_buffer = ThreadSafeBuffer(1, WorldStateReceivedTrigger) 
   sim_proto_unix_io.register_observer(
           WorldStateReceivedTrigger, world_state_received_buffer)

   while True:
       world_state_received = world_state_received_buffer.get(
               block=False, return_cached=False)
       if not world_state_received:
           world_state = tbots_protobuf.create_world_state(
               blue_robot_locations=[
                   tbots_cpp.Point(-3. y)
                   for y in numpy.linspace(-2, 2, num_robots)
               ],
               yellow_robot_locations=[
                   tbots_cpp.Point(3, y)
                   for y in numpy.linspace(-2, 2, num_robots)
               ],
               ball_location=tbots_cpp.Point(0, 0),
               ball_velocity=tbots_cpp.Vector(0, 0),
            )
            sim_proto_unix_io.send_proto(WorldState, world_state)
        else:
            break
