"""Exercise the read-only clock estimate over real HTTP, without setting time."""
import importlib.util
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
import threading
import time
import unittest

SOURCE = Path(__file__).resolve().parents[1]/'ros1/hardware/sync_clock.py'


class HardwareClockTests(unittest.TestCase):
    def test_offset_sign_comes_from_the_remote_clock(self):
        self.assertTrue(SOURCE.exists(), 'onboard clock synchronization tool is missing')
        spec = importlib.util.spec_from_file_location('hardware_clock', SOURCE)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.end_headers()
                self.wfile.write(('%.9f' % (time.time()+10.)).encode('ascii'))
            def log_message(self, *args):
                pass
        server = HTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            result = module.probe_clock('http://127.0.0.1:%d/time' % server.server_port)
            self.assertAlmostEqual(result['offset_seconds'], 10., delta=.1)
            self.assertLess(result['round_trip_seconds'], .2)
        finally:
            server.shutdown()
            thread.join()
            server.server_close()
