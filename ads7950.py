import machine
import struct
import time

class ADS7950:
    """
    pins = {
        'CS_ADC': 27,
        'CLK': 14,
        'MOSI': 13,
        'MISO': 15,
        'GPIO_ADC': 9,
    }
    chEn: list of enabled channels, i.e. chEn = [1, 1, 0, 0]
    powerdown: intended to save power, seems to make device non-responsive
    """
    def __init__(self, pins, chEn,
                 vref = 3.0, vrefRange = 1, powerdown = False,
                 spiBus = -1, spiBaud = 20e6, debug = False):
        self.pins = pins
        self.chEn = chEn
        self.vref = vref
        self.vrefRange = vrefRange
        self.powerdown = powerdown
        self.spiBus = spiBus
        self.spiBaud = spiBaud
        self.mode = None
        self.debug = debug

        # need 2 bytes for 16 bit data exchange (1 byte = 8 bits)
        self.writeBuf = bytearray(2)
        self.readBuf = bytearray(2)

        self.LSBtoV = self.vrefRange * self.vref / 2**12

        # ADS7950 datasheet pg 28:
        # SDI data is latched on every rising edge of SCLK starting
        # with the 1st clock as shown in Figure 2, Figure 3, and Figure 4.
        # data sampled on RISING edge --> phase = 0
        # FIXED issue of only reading ch0, by setting polarity = 0
        self.spi = machine.SPI(
            self.spiBus, baudrate = int(self.spiBaud),
            polarity = 0, phase = 0,
            bits = 8, firstbit = machine.SPI.MSB,
            sck = machine.Pin(pins['CLK']),
            mosi = machine.Pin(pins['MOSI']),
            miso = machine.Pin(pins['MISO'])
        )
        if self.debug: print('adc spi', self.spi)

        self.csPin = machine.Pin(self.pins['CS_ADC'],
                            mode = machine.Pin.OUT, pull = machine.Pin.PULL_UP)
        self.csPin.value(1)

        self.setMode('Manual')

        return

    def setMode(self, mode, chEn = None, lastCh = 3):
        allowedModes = ['Manual', 'Auto-1', 'Auto-2']
        if mode not in allowedModes:
            raise KeyError('mode must be in '+str(allowedModes))

        if self.debug: print('setMode ', mode)
        self.mode = mode

        if mode == 'Manual':
            self.send(self.command(mode = self.mode,
                        programming = True, channel = 0b0000,
                        vrefRange = self.vrefRange,
                        powerdown = self.powerdown,
                        gpioActive = False))

            self.readCommand = None

        elif mode == 'Auto-1':
            self.send(self.command(mode = self.mode,
                    programming = True, channel = 0b1000,
                    vrefRange = self.vrefRange, powerdown = self.powerdown,
                    gpioActive = False, gpioData = 0b0000))

            self.readCommand = self.command(mode = self.mode)

            # Frame 1, Device enters Auto-1 program sequence.
            #    Device programming is done in the next frame.
            self.send(0b1000 << 12)
            # Frame 2
            if chEn == None:
                chEn = self.chEn.copy()
            else:
                self.chEn = chEn.copy()
            chEn.reverse() # arg is [ch0, ch1, ch2, ch3], need to send MSB [ch3, ch2, ch1, ch0]
            channels = int(''.join([str(chEni) for chEni in chEn]), 2)
            self.send(channels)
            if self.debug: print('autoOneMode enabled channels ', self.chEn)

        elif mode == 'Auto-2':
            self.send(self.command(mode = self.mode,
                    programming = True, channel = 0b1000,
                    vrefRange = self.vrefRange, powerdown = self.powerdown,
                    gpioActive = False, gpioData = 0b0000))

            self.readCommand = self.command(mode = self.mode)

            if lastCh < 0 or lastCh > 3:
                raise KeyError('lastCh should be 0-3')

            din = (
                0b1001 << 12 |
                lastCh << 6
                )
            self.send(din)

        for i in range(2):
            # cycle out invalid conversion(s)
            self.read()

        return

    def gpioProgramRegister(self, alarmOutput = 'none', resetRegisters = 0, gpio3_as_powerdown = 0, gpio2_as_range_input = 0):
        """
        datasheet pg 43, Table 11
        """
        print('gpioProgramRegister')
        alarmOutputOptions = {
            'none': 0b000,
            'gpio0_high_or_low': 0b001,
            'gpio0_high_only': 0b010,
            'gpio1_low_only': 0b100,
            'gpio0_high_and_gpio1_low': 0b110
        }
        if alarmOutput not in alarmOutputOptions:
            raise KeyError('alarmOutput must be in '+str(alarmOutputOptions))

        din = (
            0b0100 << 12 | # select the GPIO Program Registers
            0b00 << 10 |
            resetRegisters << 9 |
            gpio3_as_powerdown << 8 |
            gpio2_as_range_input << 7 |
            alarmOutputOptions[alarmOutput] << 4 |
            0b0000
        )
        print(din)
        self.send(din)
        return

    def setAlarm(self, settings, group = 0, alarmOutput = 'gpio0_high_or_low'):
        """
        datasheet pg 43, Table 12
        group 0: channels 0, 1, 2, 3
        group 1: channels 4, 5, 6, 7
        group 2: channels 8, 9, 10, 11
        group 3: channels 12, 13, 14, 15
        user must manually set correct group

        "The alarm is asserted (under the alarm conditions)
        on the 12th falling edge of SCLK in the same
        frame when a data conversion is in progress.
        The alarm output is reset on the 10th falling edge
        of SCLK in the next frame."
        """
        print('setAlarm')

        groupAllowed = [0, 1, 2, 3]
        if group not in groupAllowed:
            raise KeyError('group must be in'+ str(groupAllowed))

        self.gpioProgramRegister(alarmOutput = alarmOutput)

        # Table 13. Alarm Program Register Settings
        # Frame 1, devices enters alarm programming sequence for group X
        din_1 = (
            0b11 << 14 |
            group << 12
            )
        self.send(din_1)
        time.sleep_ms(2)

        # Frame 2
        for i,setting in enumerate(settings):
            ch, threshold, highLow = setting
            threshold_b = int(threshold) & 0b1111111111 # 10-bit
            print('ch', ch, ' highLow', highLow, '  threshold', threshold, '  _b', threshold_b)
            exitAlarmProgramming = 1 if i+1 >= len(settings) else 0

            din_2 = (
                ch << 14 |
                highLow << 13 |
                exitAlarmProgramming << 12 |
                threshold_b << 0
            )
            self.send(din_2)

        # Program GPIO Register
        triggerMap = {
            'either': 0b001,
            'high': 0b010,
            'low': 0b100
            }
        din_3 = (
            0b0100 << 12 |
            0b00 << 10 |
            0 << 9 |
            0 << 8 |
            0 << 7 |
            triggerMap['either'] << 4
            )
        self.send(din_3)
        return

    def wordToAddrAndVal(self, b):
        """
        Converts a single 16 bit word into channel address and analog voltage value
        """
        if len(b) != 2:
            raise ValueError('b must be 2 bytes ' + str(b))

        resp = struct.unpack('>H', b)[0]
        # SDO outputs current channel address of the channel on DO15..12
        # followed by 12 bit conversion result on DO11..00.
        chAddr = (resp >> 12) & 0b1111 # first 4 bits
        chVal = (resp) & 0b111111111111 # last 12 bits
        #chVoltage = chVal * LSBtoV
        return (chAddr, chVal)

    def convert(self, bs):
        """
        Expects vals to be a bytearray of 1 or more 16 bit words (2 bytes)
        returns a dict of {chAddr: chVal}
        """
        i = 0
        il = len(bs)
        vals = {}

        while i < il:
            b = bs[i:i+2]
            if len(b) != 2: break
            (chAddr, chVal) = self.wordToAddrAndVal(b)
            vals.update({chAddr: chVal})
            i += 2

        return vals

    def read(self, ch = 0):
        """
        Calls subroutine according to operating mode
        Returns converted result with convert=True
        or returns bytes with convert=False
        """
        if self.mode == 'Manual':
            vals = self.readManual(ch)
        elif self.mode in ['Auto-1', 'Auto-2']:
            vals = self.readIncrementing()

        return vals

    def readIncrementing(self):
        """
        Read all channels in self.chEn
        Return the bytes result
        """
        vals = bytearray()
        for i in range(self.chEn.count(1)): # for each enabled channel
            resp = self.send(self.readCommand)
            vals.extend(resp)

        return vals

    def readManual(self, ch):
        if self.debug: print('readManual ch =', ch)
        # must discard first two SDO
        for i in range(3):
            val = self.send(self.command(mode = 'Manual', channel = ch))

        return val

    def send(self, data):
        """
        Sends the data over SPI
        Returns bytes received from device SDO
        """
        if self.debug: print('send', self.displayBinary(data))

        # > for MSB first, H for unsigned int (2 bytes)
        struct.pack_into('>H', self.writeBuf, 0, data)

        self.csPin.value(0)
        self.spi.write_readinto(self.writeBuf, self.readBuf)
        self.csPin.value(1)

        if self.debug:
            resp = struct.unpack('>H', self.readBuf)[0]
            print('resp', self.displayBinary(resp))

        return self.readBuf

    def command(self, mode = 'Manual',
                programming = False, channel = 0,
                vrefRange = 1, powerdown = False,
                gpioActive = False, gpioData = 0b0000):
        """
        Formats the data in command as binary to send over SPI
        """

        if mode == 'Manual':
            modeBit = 0b0001
        elif mode == 'Auto-1':
            modeBit = 0b0010
        elif mode == 'Auto-2':
            modeBit = 0b0011
        else:
            raise ValueError('mode ', mode, ' not valid, must be Manual, Auto-1, or Auto-2')

        if programming not in [True, False]:
            raise ValueError('programming must be True or False')

        if channel not in [0, 1, 2, 3, 0b1000]:
            raise ValueError('channel ', channel, ' out of range 0-3')

        if vrefRange not in [1, 2]:
            raise ValueError('vrefRange ', vrefRange, ' must be 1 or 2')

        if powerdown not in [True, False]:
            raise ValueError('powerdown must be True or False')

        if gpioActive not in [True, False]:
            raise ValueError('gpioActive must be True or False')

        din = (
            modeBit << 12 |
            programming << 11 |
            channel << 7 |
            (vrefRange - 1) << 6 |
            powerdown << 5 |
            gpioActive << 4 |
            gpioData << 0
            )

        return din

    def displayBinary(self, data, nSep = 4, binLen = 16, separator = ' '):
        """
        Aids debugging readability, display binary in 4-digit chunks
        """
        fixStr = ('{0:0'+str(binLen)+'b}').format(data)
        sepList = [fixStr[i:i+nSep] for i in range(0, len(fixStr), nSep)]
        return separator.join(sepList)

    def testReadSpeed(self, n = 500):
        tStart = time.ticks_us()
        for i in range(n):
            y = self.read()
        tEnd = time.ticks_us()
        tDiff = time.ticks_diff(tEnd, tStart)
        freq = n / (tDiff / 1e6)
        return freq

    def testChannelAddress(self):
        if self.debug: print('testChannelAddress')
        self.setMode('Manual')
        for reqCh in range(4):
            readDict = self.convert(self.read(reqCh))
            if reqCh not in readDict:
                raise ValueError('response did not match requested channel '
                                 +str(reqCh)+' '+str(readDict))
            else:
                print('confirmed channel addr ', readDict)

        return

    def testWithDAC(self, dac, ch = 0, dacRange = 100, fSample = 1e6):
        dac.write(0)
        self.read(ch)

        xs = []
        ys = []
        dt = 1 / fSample
        dt_us = int(dt * 1e6)

        for x in range(dacRange):
            dac.write(x)
            y = self.read(ch)
            xs.append(x)
            ys.append(y)
            time.sleep_us(dt_us)

        dac.write(0)

        print(xs)
        print(ys)
        return

    def runTests(self):
        saveMode = self.mode
        self.testChannelAddress()
        self.setMode(saveMode)
        return



if __name__ == '__main__':
    import sys

    print('RUN adc.py')

    pins = {
        'CS_ADC': 27,
        'CLK': 14,
        'MOSI': 13,
        'MISO': 15,
        'GPIO_ADC': 9,
        'EN_VBS': 4,
    }

    chEn = [1, 1, 1, 0]

    adc = ADS7950(pins = pins, chEn = chEn,
                  vref = 3.0, vrefRange = 1, powerdown = False,
                  spiBus = -1, spiBaud = 20e6, debug = True)

    adc.runTests()



    dacTests = [['DAC', 0], ['DAC2', 1]]
    for dacTest in dacTests:
        if dacTest[0] in pins:
            adc.debug = False
            adc.setMode('Manual')
            dac = machine.DAC(machine.Pin(pins[dacTest[0]]), bits = 12)
            dac.write(0)
            adc.testWithDAC(dac, ch = dacTest[1])
            adc.debug = True


    adc.debug = False
    n = 200
    adc.setMode('Manual')
    print('manual mode ', adc.testReadSpeed(n), ' Hz, ', n, 'samples ')
    adc.setMode('Auto-1', [1, 1, 1, 0])
    print('autoOne mode ', adc.testReadSpeed(n), ' Hz, ', n, 'samples ', adc.chEn, 'chEn')
    #adc.debug = True

    print(chEn)
    adc.setMode('Auto-1', chEn)

    pin_VBS = machine.Pin(pins['EN_VBS'], mode = machine.Pin.OUT)
    pin_VBS.value(1)

    def handleADCInterrupt(pin = None):
        print( str(time.time()) + ' handleADCInterrupt ' + str(pin) + '\n')
        return

    pin_GPIO_ADC = machine.Pin(pins['GPIO_ADC'],
            mode = machine.Pin.IN, pull = None)
    pin_GPIO_ADC.irq(handler = handleADCInterrupt,
                trigger = machine.Pin.IRQ_RISING)


    adc.debug = True
    #adc.setAlarm(settings = [[1, 1.67, 1] , [2, .52, 1]]) # ch, threshold, highLow
    adc.setAlarm(settings = [[1, 2250, 1]], alarmOutput = 'gpio0_high_only')
    adc.debug = False

    f = 10 # Hz
    dt = int(1/f * 1000)
    while True:
        try:
            print(adc.convert(adc.read()))

            time.sleep_ms(dt)
        except Exception as e:
            sys.print_exception(e)
            break

    adc.debug = True
