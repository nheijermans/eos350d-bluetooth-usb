#!/usr/bin/env python

import string


def pad(s, new_len, c="\x00"):
    pad_len = new_len - len(s)
    return s + pad_len * c

def printable(c):
    if c in string.printable.strip():
        return c
    else:
        return "."

def dump_hex(s, prefix="", chars_per_line=16):
    lines = (len(s) / chars_per_line)
    if len(s) % chars_per_line:
        lines += 1

    hd = ""
    for i in xrange(lines):
        data = s[i * chars_per_line:i * chars_per_line + chars_per_line]
        addr = i * chars_per_line
        s_hex = " ".join(["%02x" % ord(c) for c in data])
        s_ascii = "".join([printable(c) for c in data])
        if len(data) != chars_per_line:
            blanks = chars_per_line - len(data)
            s_hex += "   " * blanks
            s_ascii += " " * blanks

        hd += "%s%08x: %s |%s|\n" % (prefix, addr, s_hex, s_ascii)
    return hd


