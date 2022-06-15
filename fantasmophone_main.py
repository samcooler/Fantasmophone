import random, serial, time, re, glob, sys, select
# from playsound import playsound
import numpy as np
import pandas as pd
from math import log10


# Jean to do

# Read in sound table - done

# Refactor to use sensor table  - done

# Update sensor table when a button is pressed - don't need, on base

# Write sound switching procedure - done, basic
    # Write more aesthetic switching procedure

# Write looping function - don't need on base


# If time: Configuration file and more advanced sound-node assignment





class Fantasmophone:
    serial = None
    serial_port = '/dev/tty.usbmodem*'

    num_frames = 1
    num_sensors_by_frame = (48, 12, 12)
    #num_sounds = 24 now comes from sound_table_file

    rssi_by_frame = [0] * num_frames

    sound_table_file = "test_sounds.csv"
    nspiral = 2
    sensor_info_file = "test_sensors.csv"
    small_length_cutoff = 4 # sounds longer than this go to medium sensors
    nbr_margin = 1

    # Expected number of minutes between switches
    #switch_rate = 0.1 # also equal to 1/expected number of switches per minute
    

    reg_update_per_minute = 10 # Expected number of non-rotation updates per minute
    #update_rate = 1.0/reg_update_per_minute
    # For running later, set reg_update_per_minute to something like 0.1, one update every six minutes
    rotation_per_minute = 5
    #rotate_rate = 1.0/rotation_per_minute
    minutes_per_full_rotation = 5
    if rotation_per_minute  > 0:
        shift_width = nspiral/(rotation_per_minute*minutes_per_full_rotation)
    else: 
        shift_width = 0
    total_update_rate = 1.0/(reg_update_per_minute + rotation_per_minute)
    protate = rotation_per_minute*total_update_rate


    caliper = 0.1
    # Caliper controls how far away a sound can be in order to be assigned to a node

    # Out of how many cycles will we switch a sound
    # -5 is from 1e5 loops per second
    #switch_thresh = -5 - log10(60) - log10(switch_rate)

    # Number of minutes it takes to rotate back to the starting position
    minutes_per_full_rotation = 5

    # cur_sensor_values will be a list of bools with True = touched and False = not touched
    cur_sensor_values = []
    prev_sensor_values = []
    for fi in range(num_frames):
        cur_sensor_values.append([False] * num_sensors_by_frame[fi])
        prev_sensor_values.append([False] * num_sensors_by_frame[fi])

    sensors_changed_flag = False
    which_sensors_changed = []

    #led_colors = []
    #led_decays = []
    #led_periods = []

    #button_sounds = []
    #button_channels = []
    #button_repeats = []


    num_audio_channels = 6
    audio_channel_offset = 0
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

    # Not using base does
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

    # Not using base does
    def get_sensor_values(self):  # resets sensor change flag
        # assemble a package of all sensor values
        ret = {'changed': self.sensors_changed_flag,
               'which_sensors_changed': self.which_sensors_changed,  # sensor indices [0:num_sensors)
               'values': self.cur_sensor_values}
        self.sensors_changed_flag = False
        self.which_sensors_changed = []
        return ret

    # Not using this, base does this
    def play_sound(self, sound_index, channel_index, repeat=False):

        channel_index += self.audio_channel_offset
        print('play sound {} on chan {}'.format(sound_index, channel_index))

        # self.cur_playing_sounds.add(sound_index)
        # serial code to base goes here
        if self.serial is not None:
            self.serial.write(f'P{sound_index}c{channel_index}r{1*repeat}\r'.encode('utf-8'))
            self.serial.flush()

    def tx_button_sound_values(self):
        sounds = self.sensor_info.now_playing.to_list()
        channels = self.sound_table.loc[sounds, "channel"].to_list()
        loop = self.sound_table.loc[sounds, "loop"].to_list()
        gain = self.sound_table.loc[sounds, "gain"].to_list()
        out = f'V{"-".join([f"{s},{c},{r*1},{int(g)}" for s, c, r, g in zip(sounds, channels, loop, gain)])}'
        print(f'writing sound values:\n{out}')
        if self.serial is not None:
            self.serial.write(out.encode('utf-8'))
            self.serial.flush()


    def tx_led_values(self):
        sounds = self.sensor_info.now_playing.to_list()
        # todo: serial code to send LED data goes here
        colors = [int(255 * c) for c in self.sound_table.led_color[sounds].to_list()]
        periods = [int(np.clip(2 / p, 0, 15)) for p in self.sound_table.led_period[sounds].to_list()] # 2 * frequency
        decays = [int(np.clip(2 * d, 0, 15)) for d in self.sound_table.led_decay[sounds].to_list()] # 2 * decay

        led_data = zip(colors, periods, decays)
        led_data = 'L,' + ','.join([f'{l[0]:02X},{l[1]:X},{l[2]:X}' for l in led_data]) + 'X'
        print(f'LED data: {led_data}')
        if self.serial is not None:
            self.serial.write(led_data.encode('utf-8'))
            self.serial.flush()

    def read_sound_table(self):
        self.sound_table =  pd.read_csv(self.sound_table_file)
        self.num_sounds = self.sound_table.shape[0]
        # Does not do any format checking, get format right
        # Need cols loop, led_color, led_decay, led_period, sound_id, channel, gain, length_seconds, position
        self.sound_table["assigned"] = False
        self.sound_table.set_index('sound_id', inplace=True, drop=False)
        conditions = [ self.sound_table["loop"].eq(1), 
                       self.sound_table["loop"].eq(0) & self.sound_table["length_seconds"].gt(self.small_length_cutoff), 
                       self.sound_table["loop"].eq(0) & self.sound_table["length_seconds"].le(self.small_length_cutoff)]
        choices = [3, 2, 1]
        self.sound_table["sensor_size"] =  np.select(conditions, choices)


    def read_sensor_info(self):
        self.sensor_info = pd.read_csv(self.sensor_info_file)
        self.num_sensors = self.sensor_info.shape[0]
        # columns of sensor info should be 
        # sensor_id, sensor_position, sensor_size
        # sensor size should be 1, 2, or 3 (1 smallest, 2 medium, 3 large)
        self.sensor_info.set_index('sensor_id', inplace=True, drop=False)
        self.now_playing = -1


    def assign_sounds_initial(self):
        # Map sound position into the desired number of spiral loops
        xmin = min(self.sound_table.position)
        xmax = max(self.sound_table.position)
        self.sound_table["absolute_position"] = [ self.nspiral*(s-xmin)/(xmax - xmin) for s in self.sound_table.position]
        self.sound_table.position = self.sound_table.absolute_position % 1
        self.min_pos = 0
        self.max_pos = 1
        ax_sounds = self.active_sounds(self.min_pos, self.max_pos)
        #self.sound_table = self.sound_table.sort_values("position", ascending = True)

        #for ss in [1, 2, 3]:
        #    num_ss_sounds = sum(self.sound_table.sensor_size[ax_sounds].eq(ss))
        #    num_ss_sensor = sum(self.sensor_info.sensor_size.eq(ss))
        #    repl =  num_ss_sounds < num_ss_sensor 
        #    sound_id = self.sound_table.loc[ax_sounds].query(f'sensor_size == {ss}').sample(n = num_ss_sensor, replace = repl).sort_values("position", ascending = True).index
        #    sens_id = self.sensor_info.query(f'sensor_size == {ss}').sort_values("sensor_position", ascending = True).index
        #    self.sensor_info.loc[sens_id, "now_playing"] = sound_id
#
#        self.sensor_info["position"] = self.sound_table.position[self.sensor_info.now_playing].to_list()
#        self.sensor_info["absolute_position"] = self.sound_table.absolute_position[self.sensor_info.now_playing].to_list()

        # New sound assignment method respects caliper and reduces repeats
        # Does not preserve order but does preserve region
        for i in self.sensor_info.index:
            ss = self.sensor_info.sensor_size[i]
            pos = self.sensor_info.sensor_position[i]
            mycal = self.caliper
            found = False
            mysound = self.find_eligiable_sound(i, ax_sounds)
            self.sensor_info.loc[i, "now_playing"] = mysound
            self.sensor_info.loc[i, "position"] = self.sound_table.loc[mysound, "position"]
            self.sound_table.loc[mysound, "assigned"] = True

        self.sensor_info["position"] = self.sound_table.position[self.sensor_info.now_playing].to_list()
        self.sensor_info["absolute_position"] = self.sound_table.absolute_position[self.sensor_info.now_playing].to_list()

        self.sound_table.loc[self.sensor_info.now_playing, "assigned"] = True

    # Not using
    def select_swap_sound_regular(self):
        ax_sounds = self.active_sounds(self.min_pos, self.max_pos)
        n_elligiable = sum( self.sound_table.assigned[ax_sounds] == False) 
        if n_elligiable == 0:
            return -1

        sound_id_in = self.sound_table.loc[ax_sounds].query(f'assigned == False').sample(n = 1, replace = False).index
        sound_id_in = sound_id_in.to_list()[0]
        return sound_id_in


    def find_eligiable_sound(self, sensor_id, ax_sounds):
        #ax_sounds = self.active_sounds(self.min_pos, self.max_pos)
        ss = self.sensor_info.sensor_size[sensor_id]
        pos = self.sensor_info.sensor_position[sensor_id]
        mycal = self.caliper
        found = False
        print(f'Finding sound for sensor {sensor_id}\n')
        while not found:
            candidate_sounds = self.sound_table.loc[ax_sounds].query(f'sensor_size == {ss}').query(f'(( (position - {pos}) % 1) < {mycal}) or (( (position - {pos}) % 1) < {mycal})').query('assigned == False').index.to_list()
            if len(candidate_sounds) > 0:
                mysound = random.choice(candidate_sounds)
                found = True
            else:
                print("No avaialable sounds, attempting to re-use\n")
                candidate_sounds = self.sound_table.loc[ax_sounds].query(f'sensor_size == {ss}').query(f'(( (position - {pos}) % 1) < {mycal}) or (( (position - {pos}) % 1) < {mycal})').index.to_list()
                if len(candidate_sounds) > 0:
                    mysound = random.choice(candidate_sounds)
                    found = True
                else:
                    print("No sounds available in region, looking outside of region\n")
                    mycal += self.caliper
        return mysound



    # Not using
    def find_swap_mate_regular(self, sound_id_in, nbr_margin):
        # Get sound type (small/medium/large)
        new_size = self.sound_table.sensor_size[sound_id_in]
        new_pos = self.sound_table.position[sound_id_in]
        num_bigger = sum( (self.sensor_info.position >= new_pos) & (self.sensor_info.sensor_size == new_size))
        num_smaller =sum( (self.sensor_info.position < new_pos) & (self.sensor_info.sensor_size == new_size))

        if num_bigger >= nbr_margin:
            cbig = self.sensor_info.query(f'position >= {new_pos} & sensor_size == {new_size}').sort_values("position", ascending = True).sensor_id.iloc[range(nbr_margin)]
            candidate_sensors = cbig.to_list()
        elif num_bigger > 0:
            cbig = self.sensor_info.query(f'position >= {new_pos} & sensor_size == {new_size}').sort_values("position", ascending = True).sensor_id
            candidate_sensors = cbig.to_list()
        else:
            candidate_sensors = []

        if num_smaller >= nbr_margin:
            csmall = self.sensor_info.query(f'position < {new_pos} & sensor_size == {new_size}').sort_values("position", ascending = False).sensor_id.iloc[range(nbr_margin)]
            candidate_sensors.extend(csmall.to_list())
        elif num_bigger > 0:
            csmall = self.sensor_info.query(f'position < {new_pos} & sensor_size == {new_size}').sort_values("position", ascending = True).sensor_id
            candidate_sensors.extend(csmall.to_list())

        sensor_id = [random.choice(candidate_sensors)]
        return(sensor_id)
        #self.swap_sounds(sensor_id, sound_id_in)


    def swap_sounds(self, sensor_index, sound_index):
        #print(sensor_index)
        l = len(sensor_index) # sensor_index and sound_index should be the same length
        for k in range(l):
            n_id = sensor_index[k]
            o_id  = sound_index[k]
            out_o_id = self.sensor_info.now_playing[n_id] 
            pos_in = self.sound_table.position[o_id]
            abs_pos_in = self.sound_table.absolute_position[o_id]
            pos_out = self.sound_table.position[out_o_id]
            print(f'swapping sound {o_id}(position {pos_in:.2f}) onto sensor {n_id} replacing sound {out_o_id} (position {pos_out:.2f})\n')
            
            self.sensor_info.loc[n_id, "now_playing"] = o_id
            self.sensor_info.loc[n_id, "position"] = pos_in 
            self.sensor_info.loc[n_id, "absolute_position"] = abs_pos_in 

            self.sound_table.loc[o_id,"assigned"] = True
            if out_o_id not in self.sensor_info.now_playing.to_list():
                self.sound_table.loc[out_o_id,"assigned"] = False


    def shift_spiral(self, shift_width):
        old_max = self.max_pos
        self.min_pos += shift_width
        self.min_pos %= self.nspiral
        self.max_pos += shift_width
        self.max_pos %= self.nspiral

    # get sounds between min and max absolute position with wrapping
    def active_sounds(self, mn, mx):
        if(mn < mx):
                ax_sounds = self.sound_table.query(f'absolute_position >= {mn} and absolute_position < {mx}').index.to_list()
        else:
                ax_sounds = self.sound_table.query(f'absolute_position >= {mn} or absolute_position < {mx}').index.to_list()
        return ax_sounds

    def get_max_pos(self, mn, mx, ss):
        if(mn < mx):
                my_max = max(self.sensor_info.query(f'absolute_position >= {mn} and absolute_position < {mx} & sensor_size == {ss}').absolute_position.to_list())
        else:
                my_max = ax_sounds = max(self.sensor_info.query(f'absolute_position < {mx}').absolute_position.to_list())
        return my_max


    def hard_swap_after_shift(self, shift_width):
        old_min = (self.min_pos - shift_width) % self.nspiral
        old_max = (self.max_pos - shift_width) % self.nspiral
        if self.min_pos > old_min: # sensor ids with outdated sounds
            old_sens_id  = self.sensor_info.query(f'absolute_position >= {old_min} and absolute_position < {self.min_pos}').index.to_list()
            #non_old_sens_id = self.sensor_info.query(f'absolute_position < {old_min} or absolute_position > {self.min_pos}').index.to_list()
        else:
            old_sens_id  = self.sensor_info.query(f'absolute_position >= {old_min} or absolute_position < {self.min_pos}').index.to_list()
            #non_old_sens_id = self.sensor_info.query(f'absolute_position < {old_min} and absolute_position > {self.min_pos}').index.to_list()

        #self.sensor_info["temp_sensor_pos"] = (self.sensor_info.sensor_position - old_min) % self.nspiral 
        ax_sounds = self.active_sounds(self.min_pos, self.max_pos)
        for i in old_sens_id:
            mysound = self.find_eligiable_sound(i, ax_sounds)
            self.swap_sounds([i], [mysound])

        # Need to figure out how to order the sensors with outmoded sounds
        # If nbr_margin = 1, sounds should always be in order within their size group
        # So if there are non-outmoded sensors between outmoded sensors then the outmoded group spans 0, otherwise it does not
        # Could be different for the different size groups so do this within the for loop


        #for ss in [1, 2, 3]:
        #    old_sens_pos = self.sensor_info.loc[old_sens_id].query(f'sensor_size == {ss}').sort_values("sensor_position").sensor_position.to_list()
        #    print(f'There are {len(old_sens_pos)} sensors of size {ss} with outmoded sounds.\n')
        #    if len(old_sens_pos) > 0:
        #        non_old_sens_pos = self.sensor_info.loc[non_old_sens_id].query(f'sensor_size == {ss}').sort_values("sensor_position").sensor_position.to_list()
        #        contig = all([ n > max(old_sens_pos) or n < min(old_sens_pos) for n in non_old_sens_pos])
        #        if contig:
        #            old_sens_id_ss = self.sensor_info.loc[old_sens_id].query(f'sensor_size == {ss}').sort_values("sensor_position").index.to_list()
        #        else:
        #            old_sens_id_ss = self.sensor_info.loc[old_sens_id].query(f'sensor_size == {ss} & sensor_position > {max(non_old_sens_pos)}').sort_values("sensor_position").index.to_list()
        #            old_sens_id_ss.extend(self.sensor_info.loc[old_sens_id].query(f'sensor_size == {ss} & sensor_position < {min(non_old_sens_pos)}').sort_values("sensor_position").index.to_list())
#
#                # largest position currently in sensor table
#                ss_mx = self.get_max_pos(old_min, old_max, ss)
#                new_sound_id = self.active_sounds(ss_mx, self.max_pos)
#                num_sounds_ss = sum(self.sound_table.sensor_size[new_sound_id]== ss)
#                print(f'There are {num_sounds_ss} sounds of size {ss} available.\n')
#                if num_sounds_ss == 0:
#                    next
#                repl = num_sounds_ss < len(old_sens_id_ss)
#                self.sound_table["temp_sound_pos"] = (self.sound_table.position - ss_mx) % self.nspiral
##                new_sound_id_ss = self.sound_table.loc[new_sound_id].query(f'sensor_size == {ss}').sample(n = len(old_sens_id_ss), replace = repl).sort_values("temp_sound_pos", ascending = True).index.to_list()
#                self.swap_sounds(old_sens_id_ss, new_sound_id_ss)

        #self.sensor_info.drop("temp_sensor_pos", inplace = True, axis = 1)
#        self.sound_table.drop("temp_sound_pos", inplace = True, axis = 1)
                
        

    def update_sound_regular(self):
        mysens = random.choice(self.sensor_info.index.to_list())
        ax_sounds = self.active_sounds(self.min_pos, self.max_pos)
        mysound = self.find_eligiable_sound(mysens, ax_sounds)
        self.swap_sounds([mysens], [mysound])
        #sound_id_in = self.select_swap_sound_regular()
        #if(sound_id_in >=0):
        #    sens_id = self.find_swap_mate_regular(sound_id_in, nbr_margin = self.nbr_margin)
        #    self.swap_sounds(sens_id, [sound_id_in])

    def run_update_sequence(self):
        wait_time = np.random.exponential(self.total_update_rate)
        event_type = np.random.binomial(n = 1, p = self.protate, size = 1)
        print(f'Waiting {wait_time*60:.1f} seconds\n')
        time.sleep(wait_time*60)
        if event_type[0] == 0:
            print("Regular update\n")
            self.update_sound_regular()
        else:
            print("Rotation update\n")
            self.shift_spiral(self.shift_width)
            self.hard_swap_after_shift(self.shift_width)



def setup(): 
    print('Time to set up!')
    fan.initialize(enable_serial=False) 
    fan.read_sound_table()
    fan.read_sensor_info()
    fan.assign_sounds_initial()
    #fan.randomize_leds()

    #fan.randomize_sounds()
    fan.tx_button_sound_values()
    fan.tx_led_values()

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
            #else: # stop if now false and a looping sound

                # todo: modulate sounds dynamically for fun

    # todo: make some light values
    # light_magic_values = [1] * fan.num_sensors_by_frame
    # fan.set_led_values(light_magic_values, light_magic_values * 2)


if __name__ == '__main__':
    print('Starting Fantasmophone main')

    fan = Fantasmophone()
    #loop_index = 0

    setup()

    #t = time.perf_counter()
    while True:
        #loop()
        #loop_index += 1

        #if loop_index % 20000 == 0:
        #    print('FPS: {:.0f} RSSI: {}'.format(200 / (time.perf_counter() - t), fan.rssi_by_frame))
        #    t = time.perf_counter()
        #time.sleep(1 / 10000)
        fan.run_update_sequence()
        fan.tx_button_sound_values()
        fan.tx_led_values()

