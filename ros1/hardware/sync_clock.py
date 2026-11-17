#!/usr/bin/env python3
"""One-shot laptop/onboard clock alignment before LIO, Python 2/3 compatible.

Default operation only measures. --set explicitly calls GNU date on this host.
No sensor-message timestamps are modified. Use continuous NTP for long runs.
"""
from __future__ import print_function
import argparse
import json
import math
import os
import subprocess
import sys
import time
try:
    from urllib.request import build_opener, ProxyHandler
    from http.server import BaseHTTPRequestHandler, HTTPServer
except ImportError:
    from urllib2 import build_opener, ProxyHandler
    from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer


def probe_clock(url):
    samples = []
    opener = build_opener(ProxyHandler({}))
    for unused in range(5):
        start = time.time()
        response = opener.open(url, timeout=3)
        try:
            remote = float(response.read(128))
        finally:
            response.close()
        end = time.time()
        if math.isnan(remote) or math.isinf(remote):
            raise ValueError('invalid remote clock')
        samples.append(dict(offset_seconds=remote-(start+end)/2., round_trip_seconds=end-start))
    return min(samples, key=lambda s: s['round_trip_seconds'])


class TimeHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != '/time':
            self.send_error(404)
            return
        body = ('%.9f\n' % time.time()).encode('ascii')
        self.send_response(200)
        self.send_header('Content-Type', 'text/plain')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *unused):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serve', action='store_true')
    parser.add_argument('--bind', default='192.168.123.100')
    parser.add_argument('--port', type=int, default=18080)
    parser.add_argument('--url', default='http://192.168.123.100:18080/time')
    parser.add_argument('--set', action='store_true', dest='set_clock')
    args = parser.parse_args()
    if args.serve:
        server = HTTPServer((args.bind, args.port), TimeHandler)
        print('Clock reference: http://%s:%d/time (read-only)' % server.server_address)
        sys.stdout.flush()
        server.serve_forever()
        return
    if args.set_clock and os.geteuid() != 0:
        parser.error('--set requires sudo; run before starting LIO')
    result = probe_clock(args.url)
    if args.set_clock:
        subprocess.check_call(['date', '-u', '-s', '@%.9f' % (time.time()+result['offset_seconds'])])
        result = probe_clock(args.url)
    print(json.dumps(result, sort_keys=True))


if __name__ == '__main__':
    main()
