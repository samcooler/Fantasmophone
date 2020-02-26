import random


class Fantasmophone:
    num_sensors = 12
    cur_sensor_values = [False] * num_sensors
    prev_sensor_values = [False] * num_sensors
    sensors_changed_flag = False
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

        self.sensors_changed_flag = False
        for si in range(self.num_sensors):
            if self.prev_sensor_values[si] != self.cur_sensor_values:
                self.sensors_changed_flag = True
                self.which_sensors_changed.append(si)
        self.prev_sensor_values = self.cur_sensor_values.copy() # careful with list assignments sans copy

    def get_sensor_values(self):  # resets sensor change flag
        # assemble a package of all sensor values
        ret = {'changed': self.sensors_changed_flag,
               'which_sensors_changed': self.which_sensors_changed,  # sensor indices [0:num_sensors)
               'values': self.cur_sensor_values}
        self.sensors_changed_flag = False
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


# initialize global variables
fan = Fantasmophone()
loop_index = 0


def setup():
    fan.initialize()


def loop():

    if loop_index % 100 == 0:
        print('loop %d'.format(loop_index))
        # todo: add loop rate /s display

    # get serial updates, assemble fresh reporting data
    fan.update()

    # use some sensors
    sv = fan.get_sensor_values()
    if sv['changed']:
        # sensors changed so do something about it
        for si in sv['which_sensors_changed']:
            if sv['values'][si]:
                # directly map sensors to sounds by index
                audio_index = si
                fan.play_sound(audio_index, random.randint(fan.num_audio_channels))
                # todo: cache sound changes together in a list then execute in a batch

    # make some light values
    light_magic_values = [1] * fan.num_sensors




if __name__ == '__main__':
    print('Starting Fantasmophone main')

    setup()

    while True:
        loop()