import random


class Fantasmophone:
    num_sensors = 12
    cur_sensor_values = [False] * num_sensors
    prev_sensor_values = [False] * num_sensors
    sensors_changed = False
    which_sensors_changed = []

    led_intensities = [0] * num_sensors
    led_colors = [0] * num_sensors

    num_audio_channels = 2

    def initialize(self):
        print('fantasmophone init')
        # set up serial connection

    def update(self):
        print('fantasmophone update')

        # check serial for new sensor values
        # self.cur_sensor_values = ...

        self.sensors_changed = False
        for si in range(self.num_sensors):
            if self.prev_sensor_values[si] != self.cur_sensor_values:
                self.sensors_changed = True
                self.which_sensors_changed.append(si)
        self.prev_sensor_values = self.cur_sensor_values

    def get_sensor_values(self):  # resets sensor change flag
        ret = {'changed': self.sensors_changed,
               'which_sensors_changed':self.which_sensors_changed,
               'values': self.cur_sensor_values}
        self.sensors_changed = False
        self.which_sensors_changed = []
        return ret

    def play_sound(self, sound_index, channel_index):
        print('play sound %d on chan %d'.format(sound_index, channel_index))
        # serial code to wavtrigger goes here


fan = Fantasmophone()
loop_index = 0


def loop():
    print('loop')
    if loop_index % 100 == 0:
        print(loop_index)

    fan.update()

    sv = fan.get_sensor_values()
    if sv['changed']:
        # sensors changed so do something about it
        for si in sv['which_sensors_changed']:
            if sv['values'][si]:
                # directly map sensors to sounds by index
                fan.play_sound(si, random.randint(fan.num_audio_channels))


if __name__ == '__main__':
    print('Starting Fantasmophone main')

    fan.initialize()

    while True:
        loop()