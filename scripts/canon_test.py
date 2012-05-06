#!/usr/bin/env python

import binascii
import struct
import sys
import time
import usb

import canon
import utils

class canon_cmd(object):
    def __init__(self, name, cmd1, cmd2, cmd3, reply_len):
        self.name = name
        self.cmd1 = cmd1
        self.cmd2 = cmd2
        self.cmd3 = cmd3
        self.reply_len = reply_len

class canon_control_cmd(object):
    def __init__(self, name, value, cmd_len, reply_len):
        self.name = name
        self.value = value
        self.cmd_len = cmd_len
        self.reply_len = reply_len


def pack_control_subcmd(cmd, word0, word1):
    cmd_size = cmd.cmd_len - 0x10
    cmd = cmd_size * "\x00"

    header = struct.pack("<III", cmd.value, word0, word1)

    return header + cmd

def do_control_command(camera, command, a, b):
    if not isinstance(command, canon_control_cmd):
        raise Exception, "Invalid parameter."

    cmd = pack_control_subcmd(command, a, b)

    response = usb_dialogue(camera, CANON_USB_FUNCTION_CONTROL_CAMERA, cmd) 
    return response[0x50:]

class val_map(object):
    def __init__(self, map):
        self.map = map

    def get_value(self, actual_name):
        for value, name in self.map:
            if name == actual_name:
                return value
        raise AttributeError, "No such value name"

    def get_value_name(self, actual_value):
        for value, name in self.map:
            if value == actual_value:
                return name
        raise AttributeError, "Couldn't find value %s." % actual_value


class canon_release_params(object):
    def __init__(self, params):
        self.params = params

    def _set_value(self, offset, value):
        print "Setting value at offset %x to %x" % (offset, value)
        self.params = self.params[:offset] + struct.pack("B",value) + self.params[offset+1:]

    def serialize(self):
        return self.params

    def set_iso(self, value):
        map = val_map(canon.iso_map)
        self._set_value(0x1a, map.get_value(value))

    def set_aperture(self, value):
        map = val_map(canon.aperture_map)
        self._set_value(0x1c, map.get_value(value))

    def set_shutter_speed(self, value):
        map = val_map(canon.shutter_speed_map)
        self._set_value(0x1e, map.get_value(value))

    def set_beep(self, value):
        self._set_value(0x07, value)


def parse_release_params(s):
    params = [
        ("Image Format 1", 1),
        ("Image Format 2", 2),
        ("Image Format 3", 3),
        ("Self timer 1", 4),
        ("Self timer 2", 5),
        ("Flash", 6),
        ("Beep", 7),
        ("Shooting mode", 0x08),
        ("White Balance mode", 0x10),
        ("Metering mode", 0x0c),
        ("Focus mode", 0x12),
        ("ISO", 0x1a, val_map(canon.iso_map)),
        ("Aperture", 0x1c, val_map(canon.aperture_map)),
        ("Shutter speed", 0x1e, val_map(canon.shutter_speed_map)),
        ("Exposure Bias", 0x20),
    ]

    for t in params:
        if len(t) == 2:
            (name, offset) = t
            map = None
        else:
            (name, offset, map) = t

        value, = struct.unpack_from("B", s[offset:])
        if map:
            value_name = map.get_value_name(value)
            print "%s: %s (0x%02x)" % (name, value_name, value)
        else:
            print "%s: 0x%02x" % (name, value)

class canon_camera(object):
    # Supported Canon commands.
    cmd_identify = canon_cmd("Identify", 0x01, 0x12, 0x201, 0x9d)
    cmd_get_owner = canon_cmd("Get owner", 0x05, 0x12, 0x201, 0x1000)
    cmd_get_body_id = canon_cmd("EOS get body ID", 0x1d, 0x12, 0x201, 0x9c)
    cmd_get_body_id2 = canon_cmd("EOS get body ID2", 0x23, 0x12, 0x201, 0x58)
    cmd_remote_control = canon_cmd("Remote camera control", 0x13, 0x12, 0x201, 0x40)
    cmd_remote_control_new = canon_cmd("Remote camera control (new)", 0x25, 0x12, 0x201, 0x40)
    cmd_lock_keys = canon_cmd("Lock keys", 0x35, 0x12, 0x201, 0x5c)
    cmd_unlock_keys = canon_cmd("Unlock keys", 0x36, 0x12, 0x201, 0x54)
    cmd_retrieve_capture2 = canon_cmd("Download image (new)", 0x26, 0x12, 0x202, 0x40)

    # Supported control subcommands.
    cc_control_init = canon_control_cmd("Camera control init",  0x00, 0x19, 0x1c)
    cc_control_exit = canon_control_cmd("Exit release control",  0x01, 0x18, 0x1c)
    cc_release_shutter = canon_control_cmd("Release shutter", 0x04, 0x18, 0x1c)
    cc_get_release_params = canon_control_cmd("Get release params", 0x0a, 0x18, 0x4c)
    cc_set_release_params = canon_control_cmd("Set release params", 0x07, 0x3c, 0x1c)
    cc_set_transfer_mode = canon_control_cmd("Set transfer mode", 0x09, 0x1c, 0x1c)

    STATUS_ACTIVE = ord('A')
    STATUS_WOKEN = ord('C')

    REMOTE_CAPTURE_THUMB_TO_PC = 0x0001
    REMOTE_CAPTURE_FULL_TO_PC = 0x0002
    REMOTE_CAPTURE_THUMB_TO_DRIVE = 0x0003
    REMOTE_CAPTURE_FULL_TO_DRIVE = 0x0004

    def __init__(self, vid, pid):
        self.vid = vid
        self.pid = pid

        dev = None
        while dev is None:
            dev = usb.core.find(idVendor=vid, idProduct=pid)
            if not dev:
                print "Waiting for camera...\r",
                sys.stdout.flush()
        print ""
        self.dev = dev
        self.bulk_in = 0x81
        self.interrupt_in = 0x83

        self.current_sequence_number = 0
        self.dev.set_configuration()
        self.transfer_length = 0x1400
        self.init()

    def control_transfer(self, bmRequestType, bRequest, wValue, wIndex, data):
        d = self.dev.ctrl_transfer(bmRequestType, bRequest, wValue, wIndex, data)
        if bmRequestType & 0x80:
            d = "".join([chr(x) for x in d])
        return d


    def get_status(self):
        return self.control_transfer(0xc0, 0x0c, 0x0055, 0x0000, 1)

    def init(self):
        # Initialization sequence from libgphoto2/camlibs/canon/usb.c
        self.control_transfer(0x02, 0x01, 0x0000, 0x81, "")
        self.control_transfer(0x02, 0x01, 0x0000, 0x02, "")
        self.control_transfer(0x02, 0x01, 0x0000, 0x83, "")
        status = self.get_status()
        init_response = self.control_transfer(0xc0, 0x04, 0x01, 0x0000, 0x58)
        #print "status = %s" % status
        if status == "A":
            self.control_transfer(0xc0, 0x04, 0x0004, 0x0000, 0x50)
        else:
            msg = "\x10" + (0x3f * "\x00")
            msg += init_response[0x48:]
            self.control_transfer(0x40, 0x04, 0x0011, 0x0000, msg)
            data = self.dev.read(0x81, 0x44)

    def get_sequence_number(self):
        current = self.current_sequence_number
        self.current_sequence_number += 1
        return current

    sequence_number = property(get_sequence_number)

    def send_command(self, cmd, payload="", additional_reply_bytes=0):
        fmt = ("<II8x" +     # length, cmd3)
               3 * "16x" +   # More padding...
               "B3xBxBBII"   # 0x02, cmd1, 0x10, cmd2, length, sequence no.
        )

        print "Command: %s (cmd1=%x, cmd2=%x, cmd3=%x, reply=%x)" % (
                cmd.name, cmd.cmd1, cmd.cmd2, cmd.cmd3, cmd.reply_len)

        sequence = self.sequence_number

        if cmd.cmd3 == 0x202:
            byte_46 = 0x20
        else:
            byte_46 = 0x10

        length = 0x10 + len(payload)
        packet = struct.pack(fmt, length, cmd.cmd3,
                                  0x02, cmd.cmd1, byte_46, cmd.cmd2, length, sequence)

        packet += payload

        bmRequestType = 0x40
        if len(payload) > 1:
            bRequest = 0x04
        else:
            bRequest = 0x0c

        wValue = 0x0010
        wIndex = 0

        self.dev.ctrl_transfer(bmRequestType, bRequest, wValue, wIndex, packet)

        print "Sending packet (len=%x/reply=%x)" % (len(packet), cmd.reply_len)
        print utils.dump_hex(packet)
        reply = ""
        reply_bytes = cmd.reply_len + additional_reply_bytes
        if reply_bytes:
            data = self.dev.read(self.bulk_in, reply_bytes, timeout=5000)
            reply = "".join([chr(x) for x in data])
            print "Received packet (len=%x)" % len(reply)
            print utils.dump_hex(reply)

            if len(reply) >= 0x54:
                error_code, = struct.unpack_from("<50xI", reply)
                error_str = canon.error_codes.get(error_code, "Unknown error")
                if error_str:
                    print "Error code: %s (0x%x)" % (error_str, error_code)
        return reply


    def control(self, cmd, arg1=0, arg2=0, payload=""):
        size = cmd.cmd_len - 0x10

        if not payload:
            payload = struct.pack("<III", cmd.value, arg1, arg2)
            if len(payload) > size:
                payload = payload[:size]
            else:
                payload += (size - len(payload)) * chr(0)

        print "Control command: %s (value=%x, len=%x, reply=%x)" % (cmd.name,
                cmd.value, cmd.cmd_len, cmd.reply_len)
        print utils.dump_hex(payload)

        control_cmd = canon_camera.cmd_remote_control_new
        reply = self.send_command(control_cmd, payload, cmd.reply_len)

        return reply

    def interrupt_read(self, count, timeout=5000):
        data = self.dev.read(self.interrupt_in, count, timeout=timeout)
        return "".join([chr(x) for x in data])

    def control_init(self):
        return self.control(canon_camera.cc_control_init, 0, 0)

    def control_exit(self):
        return self.control(canon_camera.cc_control_exit, 0, 0)

    def control_get_release_params(self):
        params = self.control(canon_camera.cc_get_release_params, 0, 0)
        s = params[0x5c:]
        print utils.dump_hex(s)
        parse_release_params(s)
        return s

    def set_transfer_mode(self, mode):
        self.control(canon_camera.cc_set_transfer_mode, 4, mode)

    def lock_keys(self):
        return self.send_command(canon_camera.cmd_lock_keys, struct.pack("<I", 6))

    def unlock_keys(self):
        return self.send_command(canon_camera.cmd_unlock_keys)

    def get_params(self):
        self.control_init()
        params = self.control_get_release_params()
        self.control_exit()
        return params

    def set_params(self, params):
        # Might need to run the "set" twice.
        self.control_init()
        payload = struct.pack("BxxxBxxx", 0x07, 0x30) + params
        self.control(canon_camera.cc_set_release_params, payload=payload)
        self.control_exit()

    def set_aperture(self, value):
        current_params = self.get_params()
        p = canon_release_params(self.get_params())
        p.set_aperture(value)
        print "Setting new values:"
        print utils.dump_hex(p.serialize())
        self.set_params(p.serialize())

    def get_captured_image(self, key, image_size):
        CANON_DOWNLOAD_FULL = 2
        self.transfer_length = 0x00010000
        payload = struct.pack("<IIII", 0, self.transfer_length, CANON_DOWNLOAD_FULL, key)
        data = self.send_command(canon_camera.cmd_retrieve_capture2, payload)

        print utils.dump_hex(data)

        size, = struct.unpack_from("<6xI", data)

        print "Retrieving captured image (size = %u)" % size
        image = ""
        while size:
            to_read = min(size, self.transfer_length)
            data = "".join([chr(x) for x in self.dev.read(self.bulk_in, to_read)])
            image += data
            size -= len(data)
        return image


    def release_shutter(self):
        # Make sure nothing's hanging out in the interrupt pipe.
        while self.interrupt_read(0x40):
            pass

        self.control_init()
        self.set_transfer_mode(canon_camera.REMOTE_CAPTURE_FULL_TO_PC)
        self.lock_keys()
        self.control(canon_camera.cc_release_shutter, 0, 0)

        image_key = 0
        image_size = 0

        # Poll the interrupt endpoint.
        done = False
        while not done:
            reply = self.interrupt_read(0x40)

            if reply:
                print "Received %u-byte reply on interrupt pipe." % len(reply)
                print utils.dump_hex(reply)
        
            if len(reply) > 4:
                res = reply[4]

                if res == '\x0c' and len(reply) == 0x17:
                    image_key, image_size = struct.unpack_from("<IxI", reply[0x0c:])
                    self.unlock_keys()
                    done = True

            else:
                time.sleep(0.05)

        self.control_exit()

        print "Retrieving image (key=%x, size=%x)" % (image_key, image_size)
        image = self.get_captured_image(image_key, image_size)

        f = open("%08x.jpg" % image_key, "wb")
        f.write(image)
        f.close()

EOS350D_VID = 0x04a9
EOS350D_PID = 0x30ee

camera = canon_camera(EOS350D_VID, EOS350D_PID)

attempts = 4
while attempts:
    try:
        res = camera.send_command(canon_camera.cmd_identify)
        break
    except usb.core.USBError, e:
        if not attempts:
            print "Error reading body ID: %s" % str(e)
            break
        attempts -= 1

res = camera.release_shutter()
camera.set_aperture("F14")

