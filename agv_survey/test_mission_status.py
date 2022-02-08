from time import sleep
from mir_control.mir_controller import MirController


def main():
    controller = MirController([
                {'position': {'x': 10.426, 'y': 7.042}, 'orientation': -176.427},
                {'position': {'x': 4.804, 'y': 7.636}, 'orientation': -38.781}
        ])

    while True:
        sleep(1)
        print(controller.get_data())

if __name__ == '__main__':
    main()