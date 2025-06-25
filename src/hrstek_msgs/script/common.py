# -*- coding: utf-8 -*-

import hashlib
import subprocess


def format_code(code):
    res = ''
    if not code:
        return None
    if len(code) <= 4:
        return code
    for index, char in enumerate(code):
        res += char
        if (index + 1) % 4 == 0:
            res += '-'
    return res.strip('-')


def dmidecode_info(cmd, handle=True):
    p = subprocess.Popen([cmd], shell=True, stdout=subprocess.PIPE)
    data = p.stdout
    while True:
        line = data.readline()
        if isinstance(line, bytes):  # if python3
            line = line.decode()
        if line == '\n':
            break
        if line:
            if not handle:
                return line.strip("\n")
            return line.split(": ")[1].strip("\n")


def getUUID():
    return dmidecode_info("sudo dmidecode -t 1 | grep 'UUID'")


def getSerial():
    return dmidecode_info("sudo dmidecode -t 1 | grep 'Serial Number'")


def getCPU():
    return dmidecode_info("sudo dmidecode -t 4 | grep 'ID'")


def getHost():
    return dmidecode_info("hostname", handle=False)


def getMachineCode():
    result = ''
    result += getCPU() + getSerial() + getUUID() + getHost()

    md5 = hashlib.md5()
    md5.update(result.encode())
    return format_code(md5.hexdigest())
