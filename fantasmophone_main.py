import random, serial, time, re, glob, sys, select
# from playsound import playsound
import numpy as np


class Fantasmophone:
    serial = None
    serial_port = '/dev/tty.usbmodem*'

    num_frames = 1
    num_sensors_by_frame = (48, 12, 12)
    num_sounds = 24

    rssi_by_frame = [0] * num_frames
    cur_sensor_values = []
    prev_sensor_values = []
    for fi in range(num_frames):
        cur_sensor_values.append([False] * num_sensors_by_frame[fi])
        prev_sensor_values.append([False] * num_sensors_by_frame[fi])

    sensors_changed_flag = False
    which_sensors_changed = []

    led_colors = []
    led_decays = []
    led_periods = []

    button_sounds = []
    button_channels = []
    button_repeats = []


    num_audio_channels = 6
    audio_channel_offset = 2
    # cur_playing_sounds = set()

    # group1 = []

    def initialize(self, enable_serial=True):
        print('Fantasmophone init')
        # set up serial connection
        # usbmodem1423401

        if enable_serial:
            print(f'checking for serial at {self.serial_port}')
            while True:
                ports = glob.glob(self.serial_port)
                # ports = glob.glob('/dev/ttyACM*')
                if len(ports) == 0:
                    time.sleep(1)
                    print('no matching ports')
                    continue
                try:
                    self.serial = serial.Serial(ports[0], 1000000, timeout=0.5)
                    print('set up serial at {}'.format(self.serial.name))
                    break
                except serial.SerialException:
                    print('failed serial setup at {}'.format(self.serial.name))
                    time.sleep(1)
        else:
            print('No serial enabled')


    def update(self):
        # print('Fantasmophone update')

        try:
            serial_waiting = self.serial.inWaiting()
        except:
            print('Lost serial connection, trying to reestablish')
            self.initialize()
            time.sleep(1.0)
            return

        if serial_waiting > 0:

            # the below code is ugly and I want no blame for it, I am sorry but it works
            line = self.serial.readline()
            line = line.decode('utf-8')
            # print('got serial in: {}'.format(line.rstrip()))
            m = re.match('rx from (.) S: (.+)-(.+)-(.+)-(.+)-(.+)-(.+)-(.+)-(.+) RSSI (.+)\r', line)
            # print(m)
            if m:
                # print(m.groups())
                frame_index = int(m.group(1))
                self.rssi_by_frame[frame_index - 1] = int(m.group(10))

                values = []
                for i in range(8):
                    s = m.group(i + 2)
                    s = int(s, 16)
                    if i % 2 == 0:
                        n = 8
                    else:
                        n = 4
                    for si in range(n):
                        mask = 1 << si
                        values.append(s & mask != 0)

                self.cur_sensor_values[frame_index - 1] = values

                # sensor debug printout
                # todo: switch to a real logging system, make this optional
                # print('frame {}: '.format(frame_index) +
                #       ''.join(['X' if k else '_' for k in self.cur_sensor_values[frame_index-1]]))

        self.serial.flushInput()

        self.sensors_changed_flag = False
        for fi in range(self.num_frames):
            for si in range(self.num_sensors_by_frame[fi]):
                if self.prev_sensor_values[fi][si] != self.cur_sensor_values[fi][si]:
                    self.sensors_changed_flag = True
                    self.which_sensors_changed.append((fi, si))

        self.prev_sensor_values = self.cur_sensor_values.copy()  # careful with list assignments sans copy

    def get_sensor_values(self):  # resets sensor change flag
        # assemble a package of all sensor values
        ret = {'changed': self.sensors_changed_flag,
               'which_sensors_changed': self.which_sensors_changed,  # sensor indices [0:num_sensors)
               'values': self.cur_sensor_values}
        self.sensors_changed_flag = False
        self.which_sensors_changed = []
        return ret

    def play_sound(self, sound_index, channel_index, repeat=False):

        channel_index += self.audio_channel_offset
        print('play sound {} on chan {}'.format(sound_index, channel_index))

        # self.cur_playing_sounds.add(sound_index)
        # serial code to base goes here
        if self.serial is not None:
            self.serial.write(f'P{sound_index}c{channel_index}r{1*repeat}\r'.encode('utf-8'))
            self.serial.flush()

    def tx_button_sound_values(self):
        out = f'V{"-".join([f"{s},{c},{r*1}" for s, c, r in zip(self.button_sounds, self.button_channels, self.button_repeats)])}'
        print(f'writing sound values:\n{out}')
        if self.serial is not None:
            self.serial.write(out.encode('utf-8'))
            self.serial.flush()

    def randomize_sounds(self):
        self.button_sounds = [random.randint(0, self.num_sounds)+1 for a in range(self.num_sensors_by_frame[0])]
        self.button_channels = [random.randint(0, self.num_audio_channels) for a in range(self.num_sensors_by_frame[0])]
        self.button_repeats = [random.random() > 0.7 for a in range(self.num_sensors_by_frame[0])]

    def tx_led_values(self):
        # todo: serial code to send LED data goes here
        colors = [int(255 * c) for c in self.led_colors]
        periods = [int(np.clip(2 / p, 0, 15)) for p in self.led_periods] # 2 * frequency
        decays = [int(np.clip(2 * d, 0, 15)) for d in self.led_decays] # 2 * decay

        led_data = zip(colors, periods, decays)
        led_data = 'L,0,' + ','.join([f'{l[0]:02X},{l[1]:X},{l[2]:X}' for l in led_data])
        print(f'LED data: {led_data}')
        if self.serial is not None:
            self.serial.write(led_data.encode('utf-8'))
            self.serial.flush()

    def randomize_leds(self):
        self.led_colors = [random.random() for r in range(fan.num_sensors_by_frame[0])]
        self.led_decays = [random.random() * 3 for r in range(fan.num_sensors_by_frame[0])]
        self.led_periods = [random.random() * 1 for r in range(fan.num_sensors_by_frame[0])]


class SoundPalette:
    all_sounds = []
    def __init__(self):
        self.all_sounds = range(500)  # customize per palette
    def get_sound(self, index):
        return self.all_sounds[index]


def setup():
    print('Time to set up!')
    fan.initialize(enable_serial=False)

    fan.randomize_leds()
    fan.tx_led_values()

    fan.randomize_sounds()
    fan.tx_button_sound_values()


def loop():
    # if loop_index % 100 == 0:
    #     print('loop {}'.format(loop_index))
    #     # todo: add loop rate /s display

    # get serial updates, assemble fresh reporting data
    if fan.serial is not None:
        fan.update()

    # else:
        # might put some sensor simulated activations here
        # fan.simulate_buttons()


    # use some sensors
    sv = fan.get_sensor_values()
    # print(sv)
    if sv['changed']:
        print(sv)
        # sensors changed so do something about it
        for sensor_index in sv['which_sensors_changed']:

            if sv['values'][sensor_index[0]][sensor_index[1]]:  # play if now True
                # map sensors to sound index using a palette
                sound_index = pals[cur_pal].get_sound(sensor_index[1])
                print('probably playing sound')
                # fan.play_sound(sound_index, random.randint(0, fan.num_audio_channels - 1))
                # todo: cache sound changes together in a list then execute in a batch

                # todo: modulate sounds dynamically for fun

    # todo: make some light values
    # light_magic_values = [1] * fan.num_sensors_by_frame
    # fan.set_led_values(light_magic_values, light_magic_values * 2)


if __name__ == '__main__':
    print('Starting Fantasmophone main')

    fan = Fantasmophone()
    loop_index = 0

    # num_palettes = 2
    # pals = [SoundPalette() for i in range(num_palettes)]
    # cur_pal = random.randint(0, 1)

    setup()

    t = time.perf_counter()
    while True:
        loop()
        loop_index += 1

        if loop_index % 20000 == 0:
            print('FPS: {:.0f} RSSI: {}'.format(200 / (time.perf_counter() - t), fan.rssi_by_frame))
            t = time.perf_counter()
        time.sleep(1 / 10000)
