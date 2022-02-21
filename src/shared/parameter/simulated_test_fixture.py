import shared.parameter.python_bindings as py
from proto.sensor_msg_pb2 import SensorProto
from proto.world_pb2 import WorldState, SimulatorTick
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.networking.threaded_unix_sender import ThreadedUnixSender


def main():
    sensor_proto = SensorProto()
    sensor_fusion_config = py.SensorFusionConfig()
    sensor_fusion = py.SensorFusion(sensor_fusion_config)
    sensor_fusion.processSensorProto(sensor_proto)
    world = sensor_fusion.getWorld()

    attacker_tactic = py.AttackerTactic(py.AttackerTacticConfig())

    tactic_stepper = py.TacticStepper(
        attacker_tactic,
        set([py.MotionConstraint.FRIENDLY_DEFENSE_AREA]),
        py.ThunderbotsConfig(),
    )

    vision_sender = ThreadedUnixSender("/tmp/tbots/blue_vision")
    sim_init_sender = ThreadedUnixSender("/tmp/tbots/simulation_initialization")
    sim_tick_sender = ThreadedUnixSender("/tmp/tbots/simulation_tick")

    tick = SimulatorTick()
    tick.milliseconds = 10
    sim_tick_sender.send(tick)
    sim_tick_sender.send(tick)
    sim_tick_sender.send(tick)
    sim_tick_sender.send(tick)
    sim_tick_sender.send(tick)
    sim_tick_sender.send(tick)
    sim_tick_sender.send(tick)
    sim_tick_sender.send(tick)
    sim_tick_sender.send(tick)

    print(world)


if __name__ == "__main__":
    main()
