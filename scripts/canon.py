#!/usr/bin/env python

import struct

import utils

error_codes = {
    0x00000000: "",
    0x00000086: "Can't unlock EOS keys (new)",
    0x02000022: "File not found",
    0x02000029: "File was protected",
    0x0200002a: "Compact Flash card full",
    0x02000081: "Failed to lock EOS keys",
    0x02000082: "Failed to unlock EOS keys",
    0x02000085: "Could not switch to capture mode",
    0x02000086: "Invalid command parameters",
    0x02000087: "No storage card in camera",
    0x82200040: "Unknown error (new protocol)",
    0x82220040: "Unknown error (new protocol)",
}

shutter_speed_map = [
    (0x04, "bulb"),
    (0x10, "30 s"),
    (0x13, "25 s"),
    (0x15, "20 s"),
    (0x18, "15 s"),
    (0x1b, "13 s"),
    (0x1d, "10 s"),
    (0x20, "8 s"),
    (0x23, "6 s"),
    (0x25, "5 s"),
    (0x28, "4 s"),
    (0x2b, "3.2 s"),
    (0x2d, "2.5 s"),
    (0x30, "2.0 s"),
    (0x32, "1.6 s"),
    (0x35, "1.3 s"),
    (0x38, "1.0 s"),
    (0x3b, "0.8 s"),
    (0x3d, "0.6 s"),
    (0x40, "0.5 s"),
    (0x43, "0.4 s"),
    (0x45, "0.3 s"),
    (0x48, "1/4 s"),
    (0x4b, "1/5 s"),
    (0x4d, "1/6 s"),
    (0x50, "1/8 s"),
    (0x53, "1/10 s"),
    (0x55, "1/13 s"),
    (0x58, "1/15 s"),
    (0x5b, "1/20 s"),
    (0x5d, "1/25 s"),
    (0x60, "1/30 s"),
    (0x63, "1/40 s"),
    (0x65, "1/50 s"),
    (0x68, "1/60 s"),
    (0x6b, "1/80 s"),
    (0x6d, "1/100 s"),
    (0x70, "1/125 s"),
    (0x73, "1/160 s"),
    (0x75, "1/200 s"),
    (0x78, "1/250 s"),
    (0x7b, "1/320 s"),
    (0x7d, "1/400 s"),
    (0x80, "1/500 s"),
    (0x83, "1/640 s"),
    (0x85, "1/800 s"),
    (0x88, "1/1000 s"),
    (0x8b, "1/1250 s"),
    (0x8d, "1/1600 s"),
    (0x90, "1/2000 s"),
    (0x93, "1/2500 s"),
    (0x95, "1/3200 s"),
    (0x98, "1/4000 s"),
    (0x9a, "1/5000 s"),
    (0x9d, "1/6400 s"),
    (0xA0, "1/8000 s"),
]

aperture_map = [
    (0x0d, "F1.2"),
    (0x10, "F1.4"),
    (0x13, "F1.6"),
    (0x15, "F1.8"),
    (0x18, "F2.0"),
    (0x1b, "F2.2"),
    (0x1d, "F2.5"),
    (0x20, "F2.8"),
    (0x23, "F3.2"),
    (0x25, "F3.5"),
    (0x28, "F4.0"),
    (0x2b, "F4.5"),
    (0x2d, "F5.0"),
    (0x30, "F5.6"),
    (0x33, "F6.3"),
    (0x35, "F7.1"),
    (0x38, "F8"),
    (0x3b, "F9"),
    (0x3d, "F10"),
    (0x40, "F11"),
    (0x43, "F13"),
    (0x45, "F14"),
    (0x48, "F16"),
    (0x4b, "F18"),
    (0x4d, "F20"),
    (0x50, "F22"),
    (0x53, "F25"),
    (0x55, "F29"),
    (0x58, "F32"),
]

iso_map = [
    (0x40, "50"),
    (0x48, "100"),
    (0x4b, "125"),
    (0x4d, "160"),
    (0x50, "200"),
    (0x53, "250"),
    (0x55, "320"),
    (0x58, "400"),
    (0x5b, "500"),
    (0x5d, "640"),
    (0x60, "800"),
    (0x63, "1000"),
    (0x65, "1250"),
    (0x68, "1600"),
    (0x70, "3200"),
]

flash_mode_map = [
     (0x00, "Off"),
     (0x01, "On"),
     (0x02, "Auto"),
]

image_format_map = [
    (0x00, "Raw"),
    (0x01, "Raw 2"),
    (0x02, "Raw + JPEG (large, fine)"),
    (0x03, "Raw + JPEG (large, normal)"),
    (0x04, "Raw + JPEG (medium, fine)"),
    (0x05, "Raw + JPEG (medium, normal)"),
    (0x06, "Raw + JPEG (small, fine)"),
    (0x07, "Raw + JPEG (small, normal)"),
    (0x08, "JPEG (large, fine)"),
    (0x09, "JPEG (large, normal)"),
    (0x0a, "JPEG (medium, fine)"),
    (0x0b, "JPEG (medium, normal)"),
    (0x0c, "JPEG (small, fine)"),
    (0x0d, "JPEG (small, normal)"),
]

focus_mode_map = [
        (0x00, "Auto focus: one-shot"),
        (0x01, "Auto focus: AI servo"),
        (0x02, "Auto focus: AI focus"),
        (0x03, "Manual"),
]

exposure_bias_map = [
	(0x10,"+2"),
	(0x0d,"+1 2/3"),
	(0x0c,"+1 1/2"),
	(0x0b,"+1 1/3"),
	(0x08,"+1"),
	(0x05,"+2/3"),
	(0x04,"+1/2"),
	(0x03,"+1/3"),
	(0x00,"0"),
	(0xfd,"-1/3"),
	(0xfc,"-1/2"),
	(0xfb,"-2/3"),
	(0xf8,"-1"),
	(0xf5,"-1 1/3"),
	(0xf4,"-1 1/2"),
	(0xf3,"-1 2/3"),
	(0xf0,"-2"),
]

shooting_modes_map = [
	(0x00, "AUTO"),
	(0x01, "P"),
	(0x02, "Tv"),
	(0x03, "Av"),
	(0x04, "M"),
	(0x05, "A-DEP"),
	(0x06, "M-DEP"),
	(0x07, "Bulb"),
	(0x65, "Manual 2"),
	(0x66, "Far scene"),
	(0x67, "Fast shutter"),
	(0x68, "Slow shutter"),
	(0x69, "Night scene"),
	(0x6a, "Gray scale"),
	(0x6b, "Sepia"),
	(0x6c, "Portrait"),
	(0x6d, "Spot"),
	(0x6e, "Macro"),
	(0x6f, "BW"),
	(0x70, "PanFocus"),
	(0x71, "Vivid"),
	(0x72, "Neutral"),
	(0x73, "Flash off"),
	(0x74, "Long shutter"),
	(0x75, "Super macro"),
	(0x76, "Foliage"),
	(0x77, "Indoor"),
	(0x78, "Fireworks"),
	(0x79, "Beach"),
	(0x7a, "Underwater"),
	(0x7b, "Snow"),
	(0x7c, "Kids and pets"),
	(0x7d, "Night snapshot"),
	(0x7e, "Digital macro"),
	(0x7f, "MyColors"),
	(0x80, "Photo in movie"),
]

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
        return None


class release_params(object):
    def __init__(self, params):
        self.params = params

    def _set_value(self, offset, value):
        print "Setting value at offset %x to %x" % (offset, value)
        self.params = self.params[:offset] + struct.pack("B",value) + self.params[offset+1:]

    def serialize(self):
        return self.params

    def set_iso(self, value):
        map = val_map(iso_map)
        self._set_value(0x1a, map.get_value(value))

    def set_aperture(self, value):
        map = val_map(aperture_map)
        self._set_value(0x1c, map.get_value(value))

    def set_shutter_speed(self, value):
        map = val_map(shutter_speed_map)
        self._set_value(0x1e, map.get_value(value))

    def set_beep(self, value):
        self._set_value(0x07, value)

    def __str__(self):
        try:
            return parse_release_params(self.params)
        except AttributeError, e:
            s = "Couldn't parse release params: %s\n%s" % (str(e),
                    utils.dump_hex(self.params))
            return s

def parse_release_params(s):
    params = [
        ("Image Format 1", 1),
        ("Image Format 2", 2),
        ("Image Format 3", 3),
        ("Self timer 1", 4),
        ("Self timer 2", 5),
        ("Flash", 6, val_map(flash_mode_map)),
        ("Beep", 7),
        ("Shooting mode", 0x08, val_map(shooting_modes_map)),
        ("White Balance mode", 0x10),
        ("Metering mode", 0x0c),
        ("Focus mode", 0x12, val_map(focus_mode_map)),
        ("ISO", 0x1a, val_map(iso_map)),
        ("Aperture", 0x1c, val_map(aperture_map)),
        ("Shutter speed", 0x1e, val_map(shutter_speed_map)),
        ("Exposure Bias", 0x20, val_map(exposure_bias_map)),
    ]

    lines = []
    for t in params:
        if len(t) == 2:
            (name, offset) = t
            map = None
        else:
            (name, offset, map) = t

        value, = struct.unpack_from("B", s[offset:])
        value_name = None
        if map:
            value_name = map.get_value_name(value)

        if value_name:
            lines.append("%s: %s (0x%02x)" % (name, value_name, value))
        else:
            lines.append("%s: 0x%02x" % (name, value))
    return "\n".join(lines)

