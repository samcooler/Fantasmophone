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
    cur_playing_sounds = set()

    def initialize(self):
        print('Fantasmophone init')
        # set up serial connection

    def update(self):
        print('Fantasmophone update')

        # check serial for new sensor values
        # self.cur_sensor_values = ...

        self.sensors_changed = False
        for si in range(self.num_sensors):
            if self.prev_sensor_values[si] != self.cur_sensor_values:
                self.sensors_changed = True
                self.which_sensors_changed.append(si)
        self.prev_sensor_values = self.cur_sensor_values.copy() # careful with list assignments sans copy

    def get_sensor_values(self):  # resets sensor change flag
        ret = {'changed': self.sensors_changed,
               'which_sensors_changed': self.which_sensors_changed,
               'values': self.cur_sensor_values}
        self.sensors_changed = False
        self.which_sensors_changed = []
        return ret

    def play_sound(self, sound_index, channel_index):
        print('play sound %d on chan %d'.format(sound_index, channel_index))

        self.cur_playing_sounds.add(sound_index)
        # serial code to wavtrigger goes here

    def set_led_values(self, intensities, colors):
        self.led_colors = colors
        self.led_intensities = intensities

        # serial code to send LED data goes here


fan = Fantasmophone()
loop_index = 0

def setup():
    fan.initialize()

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
                audio_index = si
                fan.play_sound(audio_index, random.randint(fan.num_audio_channels))


if __name__ == '__main__':
    print('Starting Fantasmophone main')

    setup()

    while True:
        loop()