#!/usr/bin/env python3
"""
fall_server.py
--------------
Simple HTTP server that listens for fall detection POST requests
from the STM32 board and prints the alert data to the console.

Usage:
    python3 fall_server.py

Then configure your STM32 main.c with:
    SERVER_HOST  = "<your PC's IP address>"
    SERVER_PORT  = 5000
    SERVER_PATH  = "/fall-alert"
"""

from http.server import BaseHTTPRequestHandler, HTTPServer
import json
from datetime import datetime

HOST = "172.20.10.2"   # listen on all interfaces
PORT = 5000        # must match SERVER_PORT in main.c


class FallAlertHandler(BaseHTTPRequestHandler):

    def do_POST(self):
        if self.path != "/fall-alert":
            self.send_response(404)
            self.end_headers()
            return

        # Read body
        content_length = int(self.headers.get("Content-Length", 0))
        raw_body = self.rfile.read(content_length)

        # Parse JSON
        try:
            data = json.loads(raw_body.decode("utf-8"))
        except json.JSONDecodeError:
            print(f"[{timestamp()}] Received non-JSON body: {raw_body}")
            self.send_response(400)
            self.end_headers()
            return

        # Print alert
        print()
        print("=" * 50)
        print(f"  *** FALL DETECTED ***  [{timestamp()}]")
        print("=" * 50)
        print(f"  Event             : {data.get('event', 'N/A')}")
        print(f"  Accel magnitude   : {data.get('accel_mag', 'N/A')} m/s²")
        print(f"  Gyro magnitude    : {data.get('gyro_mag',  'N/A')} dps")
        print(f"  Pressure confirmed: {data.get('pressure_confirmed', 'N/A')}")
        print("=" * 50)
        print()

        # Respond with 200 OK
        response = json.dumps({"status": "ok", "received": True}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(response)))
        self.end_headers()
        self.wfile.write(response)

    def log_message(self, format, *args):
        # Suppress default Apache-style access logs — our handler prints instead
        pass


def timestamp():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


if __name__ == "__main__":
    server = HTTPServer((HOST, PORT), FallAlertHandler)
    print(f"Fall detection server running on port {PORT}")
    print(f"Waiting for POST requests on http://{HOST}:{PORT}/fall-alert ...")
    print("Press Ctrl+C to stop.\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")