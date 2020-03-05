import random, serial, time, re


class Fantasmophone:
    num_sensors = 24
    cur_sensor_values = [False] * num_sensors
    prev_sensor_values = [False] * num_sensors
    sensors_changed_flag = False
    which_sensors_changed = []

    led_intensities = [0] * num_sensors
    led_colors = [0] * num_sensors

    num_audio_channels = 2
    cur_playing_sounds = set()

    serial = None

    def initialize(self):
        print('Fantasmophone init')
        # set up serial connection

        self.serial = serial.Serial('/dev/cu.usbmodem1423401', 57600, timeout=0.5)
        print('set up serial at {}'.format(self.serial.name))

    def update(self):
        # print('Fantasmophone update')

        while self.serial.inWaiting() > 0:
            # the below code is ugly and I want no blame for it, I am sorry but it works
            line = self.serial.readline()
            line = line.decode('utf-8')
            # print('got serial in: {}'.format(line.rstrip()))
            m = re.match('S(.+)-(.+)-(.+)-(.+)\r', line)
            if m:
                self.cur_sensor_values = []
                for i in range(4):
                    s = m.group(i+1)
                    s = int(s, 16)
                    if i % 2 == 0:
                        n = 8
                    else:
                        n = 4
                    for si in range(n):
                        mask = 1 << si
                        self.cur_sensor_values.append(s & mask != 0)

                # print(''.join(['X' if k else '_' for k in self.cur_sensor_values]))

        self.serial.flushInput()

        self.sensors_changed_flag = False
        for si in range(self.num_sensors):
            if self.prev_sensor_values[si] != self.cur_sensor_values[si]:
                self.sensors_changed_flag = True
                self.which_sensors_changed.append(si)

        self.prev_sensor_values = self.cur_sensor_values.copy()  # careful with list assignments sans copy

    def get_sensor_values(self):  # resets sensor change flag
        # assemble a package of all sensor values
        ret = {'changed': self.sensors_changed_flag,
               'which_sensors_changed': self.which_sensors_changed,  # sensor indices [0:num_sensors)
               'values': self.cur_sensor_values}
        self.sensors_changed_flag = False
        self.which_sensors_changed = []
        return ret

    def play_sound(self, sound_index, channel_index):
        print('play sound {} on chan {}'.format(sound_index, channel_index))

        # self.cur_playing_sounds.add(sound_index)
        # serial code to wavtrigger goes here
        self.serial.write('P{}c{}\r'.format(sound_index, channel_index).encode('utf-8'))

    def set_led_values(self, intensities, colors):
        self.led_colors = colors
        self.led_intensities = intensities

        # todo: serial code to send LED data goes here


class SoundPalette:
    all_sounds = []

    def __init__(self):
        self.all_sounds = range(24)  # customize per palette

    def get_sound(self, index):
        return self.all_sounds[index]


def setup():
    fan.initialize()


def loop():

    # if loop_index % 100 == 0:
    #     print('loop {}'.format(loop_index))
    #     # todo: add loop rate /s display

    # get serial updates, assemble fresh reporting data
    fan.update()

    # use some sensors
    sv = fan.get_sensor_values()
    # print(sv)
    if sv['changed']:
        # sensors changed so do something about it
        for si in sv['which_sensors_changed']:
            if sv['values'][si]:  # play if now True
                # map sensors to sound index using a palette
                sound_index = pals[cur_pal].get_sound(si)
                fan.play_sound(sound_index, random.randint(0, fan.num_audio_channels - 1))
                # todo: cache sound changes together in a list then execute in a batch

                # todo: modulate sounds dynamically for fun

    # todo: make some light values
    light_magic_values = [1] * fan.num_sensors


if __name__ == '__main__':
    print('Starting Fantasmophone main')

    fan = Fantasmophone()
    loop_index = 0

    num_palettes = 2
    pals = [SoundPalette() for i in range(num_palettes)]
    cur_pal = random.randint(0, 1)

    setup()

    while True:
        loop()
        loop_index += 1
        time.sleep(1/100)