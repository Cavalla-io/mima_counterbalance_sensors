import can
import time

def receive_can_frames(channel_name, bustype, bitrate, filters):
    bus = can.interface.Bus(channel=channel_name, bustype=bustype, bitrate=bitrate, can_filters=filters)
    try:
        while True:
            msg = bus.recv(timeout=1.0)

            if msg is not None:
                raw_value = int.from_bytes(msg.data[0:2], byteorder='little', signed=True)
                steering_angle = raw_value * 0.01

                print(steering_angle)


        print(bus)
    except KeyboardInterrupt:
        print('error')
        return



def main():
    filters = filters = [{"can_id": 0x383, "can_mask": 0x7FF, "extended": False}]
    receive_can_frames('can0', 'socketcan', '250000', filters)

if __name__ == "__main__":
    main()
