#!/usr/bin/env python3
"""Thin paramiko helper to run/transfer files on the Jetson.

Usage (from PowerShell):
    "remote command"            | python _tools\\jetson.py exec
    python _tools\\jetson.py put <local> <remote>
    python _tools\\jetson.py get <remote> <local>

exec reads the whole remote command from stdin, so quoting/newlines are preserved.
Connection settings come from env JETSON_HOST/JETSON_USER/JETSON_PASS.
JETSON_PASS has no default on purpose — set it before running.
"""
import os
import paramiko
import sys

HOST = os.environ.get("JETSON_HOST", "192.168.8.100")
USER = os.environ.get("JETSON_USER", "nvidia")
PASS = os.environ.get("JETSON_PASS", "")


def connect():
    if not PASS:
        raise SystemExit("环境变量 JETSON_PASS 未设置，请先设置后再运行")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=10)
    return c


def exec_cmd(c):
    import time
    cmd = sys.stdin.read()
    i, o, e = c.exec_command(cmd, timeout=900)
    ch = o.channel
    ch.settimeout(1.0)
    while True:
        try:
            data = ch.recv(65536)
        except OSError:
            continue
        if not data:
            break
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()
    rc = ch.recv_exit_status()
    err = e.read().decode("utf-8", "replace")
    if err:
        print("[STDERR]\n" + err, end="")
    return rc


def put(f):
    c = connect()
    c.open_sftp().put(sys.argv[2], sys.argv[3])
    c.close()
    return 0


def get(f):
    c = connect()
    c.open_sftp().get(sys.argv[2], sys.argv[3])
    c.close()
    return 0


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    mode = sys.argv[1]
    c = connect()
    try:
        if mode == "exec":
            return exec_cmd(c)
        if mode == "put":
            return put(c)
        if mode == "get":
            return get(c)
        print(__doc__)
        return 2
    finally:
        c.close()


if __name__ == "__main__":
    sys.exit(main())
