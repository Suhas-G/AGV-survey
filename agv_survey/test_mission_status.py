from time import sleep
from mir_control.mir_controller import MirController


def main():
    controller = MirController([
                {'position': {'x': 11.050, 'y': 9.450}, 'orientation': 180},
                {'position': {'x': 7.239, 'y': 8.150}, 'orientation': 0}
        ])

    while True:
        sleep(1)
        # controller.get_data()
        print('Data: ', controller.get_data())

if __name__ == '__main__':
    main()