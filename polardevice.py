#!/usr/bin/env python3

import gatt
import struct
import math
from enum import Enum
from threading import Timer

class PolarDevice(gatt.Device):
    """
    PolarDevice is a sub-class of gatt.Device() that handles
    the connection to a Polar OH1 device.
    """
    CP_CHAR_UUID = "fb005c81-02e7-f387-1cad-8acd2d8df0c8"
    DATA_CHAR_UUID = "fb005c82-02e7-f387-1cad-8acd2d8df0c8"
    HR_CHAR_UUID = "00002a37-0000-1000-8000-00805f9b34fb"
    HR_SERVICE_UUID = "0000180d-0000-1000-8000-00805f9b34fb"
    POLAR_SERVICE_UUID = "fb005c80-02e7-f387-1cad-8acd2d8df0c8"

    class CPErrorCode(Enum):
        """
        Enum representing the various ErrorCodes
        """
        SUCCESS = 0
        INVALID_OP_CODE = 1
        INVALID_MEASUREMENT_TYPE = 2
        NOT_SUPPORTED = 3
        INVALID_LENGTH = 4
        INVALID_PARAMETER = 5
        INVALID_STATE = 6
        INVALID_RESOLUTION = 7
        INVALID_SAMPLE_RATE = 8
        INVALID_G_RATE = 9
        INVALID_MTU = 10

    class CPOpCode(Enum):
        """
        Enum representing the various available OpCodes
        """
        START_MEASUREMENT = 2
        STOP_MEASUREMENT = 3

    class MeasurementType(Enum):
        """
        Enum representing the various measurement types available
        """
        ECG = 0  # Electrocardiography Data
        PPG = 1  # Photoplethysmography Data
        ACC = 2  # Accelerometer Data
        PPI = 3  # Peak to peak interval (from PPG-data)

    def __init__(self, mac_address, manager, managed=True, debug=False):
        super().__init__(mac_address, manager, managed)
        self._cp_char = None
        self._data_char = None
        self._hr_char = None
        self.__debug = debug
        self.__ppi_restart_inprogress = False
        self.__ppi_start_timer = None
        self.__ppi_restart_timer = None

    def connect_succeeded(self):
        super().connect_succeeded()
        if self.__debug:
            print("[%s] Connected" % (self.mac_address))
        self.manager.on_device_connected()

    def connect_failed(self, error):
        super().connect_failed(error)
        if self.__debug:
            print("[%s] Connection failed: %s" % (self.mac_address, str(error)))

    def disconnect_succeeded(self):
        super().disconnect_succeeded()
        if self.__debug:
            print("[%s] Disconnected" % (self.mac_address))
        self.manager.on_device_disconnected()

    def services_resolved(self):
        super().services_resolved()
        if self.__debug:
            print("[%s] Resolved services" % (self.mac_address))
        for service in self.services:
            # print("[%s]  Service [%s]" % (self.mac_address, service.uuid))
            if service.uuid == PolarDevice.POLAR_SERVICE_UUID:
                for characteristic in service.characteristics:
                    # print("[%s]    Characteristic [%s]" % (self.mac_address, characteristic.uuid))
                    characteristic.enable_notifications()
                    if characteristic.uuid == PolarDevice.DATA_CHAR_UUID:
                        self._data_char = characteristic
                    elif characteristic.uuid == PolarDevice.CP_CHAR_UUID:
                        self._cp_char = characteristic

                self._enable_ppi()

    def characteristic_value_updated(self, characteristic, value):
        """
        Called when a characteristic value has changed.
        """
        if characteristic.uuid == PolarDevice.DATA_CHAR_UUID:
            self._parse_data(value)
        elif characteristic.uuid == PolarDevice.CP_CHAR_UUID:
            self._parse_cp_response(value)

    def characteristic_write_value_succeeded(self, characteristic):
        """
        Called when a characteristic value write command succeeded.
        """
        pass

    def _enable_ppi(self):
        """
        Tell the OH-1 that it should start to stream PPI values
        """
        cmd = []
        cmd.append(PolarDevice.CPOpCode.START_MEASUREMENT.value)
        cmd.append(PolarDevice.MeasurementType.PPI.value)

        # The OH-1 seems to be a bit unstable when enabling PPI
        # but usually works ok if the call is delayed a bit.
        if self.__ppi_start_timer:
            self.__ppi_start_timer.cancel()
        self.__ppi_start_timer = Timer(5.0, lambda: self._cp_char.write_value(cmd))
        self.__ppi_start_timer.start()

        self.__ppi_restart_inprogress = False

        # Start a timer that will restart PPI measurement after 19 min
        # since there's a bug that causes measurement to stop after 20 min
        # See https://github.com/polarofficial/polar-ble-sdk/issues/43
        if self.__ppi_restart_timer:
            self.__ppi_restart_timer.cancel()
        self.__ppi_restart_timer = Timer(19*60, lambda: self._restart_ppi())
        self.__ppi_restart_timer.start()

    def _disable_ppi(self):
        """
        Tell the OH-1 that it should stop streaming PPI values
        """
        cmd = []
        cmd.append(PolarDevice.CPOpCode.STOP_MEASUREMENT.value)
        cmd.append(PolarDevice.MeasurementType.PPI.value)
        self._cp_char.write_value(cmd)

    def _restart_ppi(self):
        if self.__debug:
            print("[%s] Restarting PPI measurement" % self.mac_address)
        self.__ppi_restart_inprogress = True
        self._disable_ppi()

    def _parse_data(self, data):
        """
        Determine data type, and call appropriate handler function
        """
        type1, _, type2 = struct.unpack("<BqB", data[0:10])

        if type1 == PolarDevice.MeasurementType.PPI.value:
            self._parse_ppi(data)
        elif type1 == PolarDevice.MeasurementType.PPG.value:
            self._parse_ppg(data)
        else:
            print("Unhandled data received with type: {}".format(str(int(data[0]))))

    def _parse_ppg(self, data):
        # type1 (8)
        # timestamp (64)
        # type2 (8)
        # list of sample, where each is:
        #   ppg1 (24)
        #   ppg2 (24)
        #   ppg3 (24)
        #   amb (24)

        if self.__debug:
            print("Data length: {}".format(len(data)))

        def get_ppg_value(subdata):
            # A bit of magic happening here with the padding.
            # Since the value comes as a 24 bit signed int, it's padded to allow the use
            # of struct.unpack("<i") since that takes a 32 bit signed integer.
            # The padding is then either 0xFF or 0x00 depending of if the most significant
            # bit of the most significant byte of the 24 bit value was set. Since this
            # determine if the value was positive of negative.
            return struct.unpack("<i", subdata + (b'\0' if subdata[2] < 128 else b'\xff'))[0]

        numSamples = math.floor((len(data) - 10) / 12)
        for x in range(numSamples):
            for y in range(4):
                if self.__debug:
                    print("PPG Value {}: {}".format(y,
                                                    get_ppg_value(data[10 + x * 12 + y * 3:(10 + x * 12 + y * 3) + 3])))

    def _enable_ppg(self):
        """
        Tell the OH-1 that it should start to stream PPG values
        """
        cmd = []
        cmd.append(PolarDevice.CPOpCode.START_MEASUREMENT.value)
        cmd.append(PolarDevice.MeasurementType.PPG.value)
        cmd.append(0x00)  # Sample rate Setting
        cmd.append(0x01)  # array count (?)
        cmd.append(0x82)  # 16 bit value: 130 Hz
        cmd.append(0x00)  # see above
        cmd.append(0x01)  # Resolution Setting
        cmd.append(0x01)  # array count
        cmd.append(0x16)  # 16 bit value: 22 bit
        cmd.append(0x00)  # see above
        self._cp_char.write_value(cmd)

    def _parse_ppi(self, data):
        """
        Parse and pass on PPI data, including heart rate
        """
        # type (8)
        # timestamp (64)
        # type2 (8)
        # loop for remaining bytes ((value.length - 1 - 8 - 1) /6)
        # heartrate (8)
        # ppi (16)
        # errEst (16)
        # flags (8)

        type1, _, type2 = struct.unpack("<BqB", data[0:10])
        if type1 == 3:
            numSamples = math.floor((len(data) - 8) / 6)

            samples = []
            for x in range(numSamples):
                start = 10 + x * 6
                sample = data[start:start + 6]
                hr, ppi, errEst, flags = struct.unpack("<BHHB", sample)
                # print("HR: {}\t PPI: {}, Error Est.: {}".format(hr, ppi, errEst))
                sample = {
                    'heart_rate': hr,
                    'ppi': ppi,
                    'error_estimate': errEst,
                }
                samples.append(sample)
            self._output_sample(samples)


    def _parse_cp_response(self, data):
        """
        Parse incoming data on the control point characteristic
        """
        # A response frame looks like this:
        # type (8) (F0 = CP response)
        # opcode (8)
        # param (?)
        # error_code(8)
        # more_frames(8)
        # reserved (8) (only for Start Meas, not present for Stop Meas)
        response_type = int.from_bytes(data[0:1], byteorder="little")
        opcode = int.from_bytes(data[1:2], byteorder="little")
        # print("OpCode: {}".format(int.from_bytes(data[1:2], byteorder="little")))
        if response_type == 0xF0 and opcode == PolarDevice.CPOpCode.START_MEASUREMENT.value:
            response = PolarDevice.CPErrorCode(struct.unpack("<B", data[3:4])[0])
            if response == PolarDevice.CPErrorCode.SUCCESS:
                if self.__debug:
                    print("Successfully started PPI measurement")
            elif response == PolarDevice.CPErrorCode.INVALID_STATE:
                if self.__debug:
                    print("PPI measurement already started")
        elif response_type == 0xF0 and opcode == PolarDevice.CPOpCode.STOP_MEASUREMENT.value:
            response = PolarDevice.CPErrorCode(struct.unpack("<B", data[3:4])[0])
            if response == PolarDevice.CPErrorCode.SUCCESS:
                if self.__ppi_restart_inprogress:
                    self._enable_ppi()
                if self.__debug:
                    print("Successfully stopped PPI measurement")
            elif response == PolarDevice.CPErrorCode.INVALID_STATE:
                if self.__debug:
                    print("PPI measurement already stopped")

    def _output_sample(self, sample):
        self.manager._on_device_samples(sample)


class PolarDeviceManager(gatt.DeviceManager):
    def __init__(self,
                 adapter_name,
                 device_name_filter="",
                 device_address="",
                 debug=False):
        self._device_name_filter = device_name_filter
        self._device_address = device_address
        self._device = None
        self._callbacks = []
        self.__debug = debug
        self.__stopping = False
        super().__init__(adapter_name)

    def make_device(self, mac_address):
        return PolarDevice(mac_address=mac_address, manager=self, debug=self.__debug)

    def device_discovered(self, device):
        if self._device_already_connected():
            return

        if self._device_matches_address(device) or self._device_matches_name_filter(device):
            if self.__debug:
                print("Found device: {}".format(device.alias()))
            self._device = device
            device.connect()

    def on_device_disconnected(self):
        self._device = None
        if not self.__stopping:
            self.start_discovery()

    def on_device_connected(self):
        self.stop_discovery()

    def add_callback(self, cb):
        """
        Add a callback which is called when new data is available. A dict
        containing the data will be passed as a parameter.
        """
        if cb not in self._callbacks:
            self._callbacks.append(cb)

    def remove_callback(self, cb):
        """
        Remove a callback
        """
        if cb in self._callbacks:
            self._callbacks.remove(cb)

    def _on_device_samples(self, samples):
        """
        Called by the devices when they get new data. This function is
        responsible for calling the registered callbacks.
        """
        for cb in self._callbacks:
            cb(samples)

    def stop(self):
        self.__stopping = True
        super().stop()
        if self._device:
            self._device.disconnect()

    def run(self):
        self.__stopping = False
        self._device = None
        super().run()

    def _device_matches_address(self, device):
        if self._device_address and device.mac_address == self._device_address:
            return True
        else:
            return False

    def _device_matches_name_filter(self, device):
        if (device.alias() and self._device_name_filter and
            self._device_name_filter in device.alias()):
            return True
        else:
            return False

    def _device_already_connected(self):
        return True if self._device else False        


if __name__ == "__main__":
    print("This is intended to be used as a module")
    print("Will now launch a basic test scenario")

    def print_callback(sample):
        print("PrintCallback: {}".format(repr(sample)))


    manager = PolarDeviceManager(adapter_name='hci0', device_name_filter="Polar OH1", debug=True)
    manager.start_discovery()
    manager.add_callback(print_callback)
    manager.run()
