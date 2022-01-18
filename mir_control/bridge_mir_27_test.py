import mirrosbridge as mb

from .config import MIR_ROS_HOST, MIR_ROS_PORT


if __name__ == "__main__":
    try:
        connection = mb.MirRosBridge(MIR_ROS_HOST, MIR_ROS_PORT)
        connection.set_continous_manual_control(True)
        connection.move(mb.MirRosBridge.turn_right()[0], mb.MirRosBridge.turn_right()[1])

    except (
        RuntimeError,
        TypeError,
        NameError,
        KeyboardInterrupt,
        AttributeError,
    ) as e:
        print("error: %s" % e)
        connection.shutdown()
