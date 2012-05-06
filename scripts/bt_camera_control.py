#!/usr/bin/env python

import argparse
import binascii
import os
import re
import serial
import struct
import sys

import canon
import utils

# NOTE on reading from the serial port:
#  When reading directly from /dev/rfcomm0 on Linux (using open()), this script
#  ends up missing a small amount of data that the board on the other side
#  of the Bluetooth connection tries to send. Using the pyserial module
#  alleviates this problem for some reason - I haven't looked into the reasons
#  why yet.

CMD_MAGIC = "CCMD"
CMD_GET_STATUS = 0x30
CMD_TRIGGER_SHUTTER_RELEASE = 0x31
CMD_GET_IMAGE = 0x32
CMD_GET_RELEASE_PARAMS = 0x33
CMD_SET_RELEASE_PARAMS = 0x34

STATUS_OK = 0x00
STATUS_CAMERA_DISCONNECTED = 0x80
STATUS_GENERAL_ERROR = 0xff

class camera_error(Exception):
    def __str__(self):
        return "An error occurred while communicating with the camera."

class camera_not_connected_error(camera_error):
    def __str__(self):
        return "No camera is connected."

class camera_obj(object):
    def __init__(self, port):
        self.h = serial.Serial(port=port)

    def send_command(self, cmd_id, data=""):
        cmd = struct.pack(">4sIB", CMD_MAGIC, len(data), cmd_id) + data
        self.h.write(cmd)

        response_header = self.h.read(10)

        (magic, cmd_id, status, data_len) = \
                struct.unpack_from(">4sBBI", response_header)

        if status == STATUS_OK:
            return self.h.read(data_len)
        else:
            if status == STATUS_CAMERA_DISCONNECTED:
                raise camera_not_connected_error, status
            else:
                raise camera_error, status

    def get_status(self):
        self.send_command(CMD_GET_STATUS)

    def get_release_params(self):
        data = self.send_command(CMD_GET_RELEASE_PARAMS)
        return canon.release_params(data)

    def set_aperture(self, value):
        params = self.get_release_params()
        self.send_command(CMD_SET_RELEASE_PARAMS, params.serialize())

    def release_shutter(self):
        reply = self.send_command(CMD_TRIGGER_SHUTTER_RELEASE)
        image_key, image_size = struct.unpack_from(">II", reply)
        return (image_key, image_size)

    def get_image(self, image_key, image_size):
        # Note: Reading 0x1000 bytes at a time appears to overflow a buffer on
        # the RN42 Bluetooth dongle. The chunk size should be small enough to
        # prevent this from happening; 0x500 seems to work fine.
        print "Requesting image %u (size=%u)" % (image_key, image_size)
        chunk_size = 0x500
        img = ""
        i = 0
        filename = "%08x.jpg" % image_key

        f = open(filename, "wb")

        while image_size:
            to_read = min(image_size, chunk_size)
            params = struct.pack(">II", image_key, to_read)
            data = self.send_command(CMD_GET_IMAGE, params)

            f.write(data)

            image_size -= len(data)

            print "Read chunk %u: %u bytes (%u remain): %s\r" % (i, len(data),
                    image_size, binascii.hexlify(data[:8])),
            sys.stdout.flush()
            i += 1
        f.close()
        print "\nImage saved to %s." % filename


def main():
    parser = argparse.ArgumentParser("Control a camera attached to a Bluetooth SPP port.")
    parser.add_argument("-p", "--port", default="/dev/rfcomm0", dest="port")
    parser.add_argument("-i", "--info", action="store_true", dest="get_info")
    parser.add_argument("-c", "--capture", action="store_true", dest="capture")

    args = parser.parse_args()

    camera = camera_obj(args.port)

    try:
        camera.get_status()

        if args.get_info:
            print "Camera release parameters"
            print "-------------------------"
            print "%s" % camera.get_release_params()

        if args.capture:
            (key, size) = camera.release_shutter()
            camera.get_image(key, size)

    except camera_error, e:
        print "%s" % e


if __name__ == "__main__":
    main()

