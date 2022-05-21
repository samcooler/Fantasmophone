import random, serial, time, re, glob, sys, select
# from playsound import playsound


class Fantasmophone:
    num_frames = 3
    num_sensors_by_frame = (48, 12, 12)

    rssi_by_frame = [0] * num_frames
    cur_sensor_values = []
    prev_sensor_values = []
    for fi in range(num_frames):
        cur_sensor_values.append([False] * num_sensors_by_frame[fi])
        prev_sensor_values.append([False] * num_sensors_by_frame[fi])

    sensors_changed_flag = False
    which_sensors_changed = []

    led_intensities = []
    led_colors = []

    for fi in range(num_frames):
        led_intensities.append([0] * num_sensors_by_frame[fi])
        led_colors.append([0] * num_sensors_by_frame[fi])

    num_audio_channels = 2
    audio_channel_offset = 0
    cur_playing_sounds = set()

    serial = None

    # group1 = []

    def initialize(self):
        print('Fantasmophone init')
        # set up serial connection
        # usbmodem1423401

        while True:
            ports = glob.glob('/dev/tty.usbmodem142201')
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
            print('got serial in: {}'.format(line.rstrip()))
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

    def play_sound(self, sound_index, channel_index):

        channel_index += self.audio_channel_offset
        print('play sound {} on chan {}'.format(sound_index, channel_index))

        # self.cur_playing_sounds.add(sound_index)
        # serial code to base goes here
        self.serial.write('P{}c{}\r'.format(sound_index, channel_index).encode('utf-8'))
        self.serial.flush()

    def set_led_values(self, intensities, colors):
        self.led_colors = colors
        self.led_intensities = intensities

        # todo: serial code to send LED data goes here
        led_data = zip(self.led_colors, self.led_intensities)
        led_data = [item for sublist in led_data for item in sublist]
        print('LED data: {}'.format(led_data))
        self.serial.write('L{}'.format(led_data).encode('utf-8'))


class SoundPalette:
    all_sounds = []

    def __init__(self):
        self.all_sounds = range(500)  # customize per palette

    def get_sound(self, index):
        return self.all_sounds[index]


def setup():
    fan.initialize()
    print('Time to set up!')
    # print('Touch group 1 sensors. When you are done hit enter.')
    # while True:
    #     fan.update()
    #     sv = fan.get_sensor_values()
    #     if sv['changed']:
    #         # print(sv)
    #         # sensors changed so do something about it
    #         for sensor_index in sv['which_sensors_changed']:
    #             print(sensor_index[1])
    #             fan.group1.append(sensor_index)
    #     time.sleep(1 / 100)
    #     if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
    #         line = input()
    #         break
    # print('Group 1 sensors: {}'.format(fan.group1))


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
        print(sv)
        # sensors changed so do something about it
        for sensor_index in sv['which_sensors_changed']:

            if sv['values'][sensor_index[0]][sensor_index[1]]:  # play if now True
                # map sensors to sound index using a palette
                sound_index = pals[cur_pal].get_sound(sensor_index[1])
                fan.play_sound(sound_index, random.randint(0, fan.num_audio_channels - 1))
                # todo: cache sound changes together in a list then execute in a batch

                # todo: modulate sounds dynamically for fun

    # todo: make some light values
    # light_magic_values = [1] * fan.num_sensors_by_frame
    # fan.set_led_values(light_magic_values, light_magic_values * 2)


if __name__ == '__main__':
    print('Starting Fantasmophone main')

    fan = Fantasmophone()
    loop_index = 0

    num_palettes = 2
    pals = [SoundPalette() for i in range(num_palettes)]
    cur_pal = random.randint(0, 1)

    setup()

    t = time.perf_counter()
    while True:
        loop()
        loop_index += 1

        if loop_index % 200 == 0:
            print('FPS: {:.0f} RSSI: {}'.format(200 / (time.perf_counter() - t), fan.rssi_by_frame))
            t = time.perf_counter()
        time.sleep(1 / 100)
