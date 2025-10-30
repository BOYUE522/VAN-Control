#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import base64
import socket
import errno
import time
import threading
import select
import sys

import queue as Queue

# Global variables
rtcm_queue = Queue.Queue()
gga_queue = Queue.Queue()


class NtripSocketThread (threading.Thread):
    def __init__(self, caster_ip, caster_port, mountpoint, username, password, roslogger):
        threading.Thread.__init__(self)
        self.stop_event = threading.Event()
        self.no_rtcm_data_count = 0
        self.sent_gga = False
        self.ntrip_tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connected_to_caster = False
        self.username = username
        self.password = password
        self.mountpoint = mountpoint
        self.caster_ip = caster_ip
        self.caster_port = caster_port
        self.roslogger = roslogger

    def connect_to_ntrip_caster(self):
        self.roslogger.info('Connecting to NTRIP caster at %s:%d' % (self.caster_ip, self.caster_port))

        try:
            self.ntrip_tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.ntrip_tcp_sock.settimeout(5.0)
            self.ntrip_tcp_sock.connect((self.caster_ip, self.caster_port))
            self.ntrip_tcp_sock.settimeout(None)
            self.roslogger.info('Successfully opened socket')
        except Exception as ex:
            self.roslogger.info('Error connecting socket: %s' % ex)
            self.ntrip_tcp_sock.settimeout(None)
            return False

        encoded_credentials = base64.b64encode((self.username + ':' + self.password).encode('ascii'))
        if (sys.version_info > (3, 0)):
            server_request = ('GET /%s HTTP/1.0' % self.mountpoint).encode('utf-8') + b'\x0D' + b'\x0A' \
                            + 'User-Agent: NTRIP ABC/1.2.3'.encode('utf-8') + b'\x0D' + b'\x0A' \
                            + 'Accept: */*'.encode('utf-8') + b'\x0D' + b'\x0A' \
                            + 'Connection: close'.encode('utf-8') + b'\x0D' + b'\x0A' \
                            + 'Authorization: Basic '.encode('utf-8') + encoded_credentials + b'\x0D' + b'\x0A' + b'\x0D' + b'\x0A'
        else:
            server_request = 'GET /%s HTTP/1.0\r\nUser-Agent: NTRIP ABC/1.2.3\r\nAccept: */*\r\nConnection: close\r\nAuthorization: Basic %s\r\n\r\n' % (self.mountpoint, encoded_credentials)

        self.ntrip_tcp_sock.sendall(server_request)

        while True:
            try:
                response = self.ntrip_tcp_sock.recv(10000)
            except socket.error as e:
                err = e.args[0]
                if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                    continue
                else:
                    # a "real" error occurred
                    self.roslogger.info(e)
                    return False
            else:
                if ('ICY 200 OK').encode() in response:
                    self.roslogger.info('Successfully connected to NTRIP caster')
                    return True
                else:
                    self.roslogger.info('Received unexpected response from caster:\n%s' % response)
                    return False

    def run(self):
        self.roslogger.info('Starting NTRIP TCP socket thread')
        while not self.stop_event.isSet():

            if not self.connected_to_caster:
                if self.connect_to_ntrip_caster():
                    self.connected_to_caster = True
                else:
                    time.sleep(0.05)
                    continue

            # Receive RTCM messages from NTRIP caster and put in queue to send to GPS receiver
            try:
                ready_to_read, ready_to_write, in_error = select.select([self.ntrip_tcp_sock, ], [self.ntrip_tcp_sock, ], [], 5)
            except select.error:
                self.ntrip_tcp_sock.close()
                self.connected_to_caster = False
                self.roslogger.info('Error calling select(): resetting connection to NTRIP caster')
                continue

            if len(ready_to_read) > 0:
                rtcm_msg = self.ntrip_tcp_sock.recv(100000)
                if len(rtcm_msg) > 0:
                    if (sys.version_info > (3, 0)):
                        if rtcm_msg[0] == 211:
                            rtcm_msg_len = 256 * rtcm_msg[1] + rtcm_msg[2]
                            rtcm_msg_no = (256 * rtcm_msg[3] + rtcm_msg[4]) / 16
                            self.roslogger.info('Received RTCM message %d with length %d' % (rtcm_msg_no, rtcm_msg_len))
                            if rtcm_msg_no == 1029:
                                self.roslogger.info(f'Message from NTRIP server: {rtcm_msg[11:3+rtcm_msg_len]}')
                    else:
                        if ord(rtcm_msg[0]) == 211:
                            rtcm_msg_len = 256 * ord(rtcm_msg[1]) + ord(rtcm_msg[2])
                            rtcm_msg_no = (256 * ord(rtcm_msg[3]) + ord(rtcm_msg[4])) / 16
                            self.roslogger.info('Received RTCM message %d with length %d' % (rtcm_msg_no, rtcm_msg_len))

                    rtcm_queue.put(rtcm_msg)
                    self.no_rtcm_data_count = 0

            # Get GPGGA messages from receive queue and send
            # to NTRIP server to keep connection alive
            if len(ready_to_write) > 0:
                try:
                    gga_msg = gga_queue.get_nowait()
                    self.roslogger.info('Sending new GGA message to NTRIP caster %s' % gga_msg)
                    self.ntrip_tcp_sock.send(gga_msg.encode('utf-8'))
                    self.sent_gga = True
                except Queue.Empty:
                    pass

            if self.no_rtcm_data_count > 200:
                self.roslogger.info('No RTCM messages for 10 seconds; resetting connection to NTRIP caster')
                self.ntrip_tcp_sock.close()
                self.connected_to_caster = False
                self.no_rtcm_data_count = 0

            if self.sent_gga:
                self.no_rtcm_data_count += 1

            time.sleep(0.05)

        self.roslogger.info('Stopping NTRIP TCP socket thread')
        self.ntrip_tcp_sock.close()

    def stop(self):
        self.stop_event.set()


class ReceiverThread (threading.Thread):
    def __init__(self, broadcast_ip, broadcast_port, roslogger):
        threading.Thread.__init__(self)
        self.stop_event = threading.Event()
        self.receiver_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.broadcast_ip = broadcast_ip
        self.broadcast_port = broadcast_port
        self.roslogger = roslogger

    def run(self):
        self.roslogger.info('Starting relay socket thread')
        while not self.stop_event.isSet():
            # Get RTCM messages from NTRIP TCP socket queue and send to GPS receiver over UDP
            try:
                rtcm_msg = rtcm_queue.get_nowait()
                self.roslogger.info('Broadcasting RTCM message to %s:%d' % (self.broadcast_ip, self.broadcast_port))
                self.receiver_sock.sendto(rtcm_msg, (self.broadcast_ip, self.broadcast_port))
            except Queue.Empty:
                # Nothing in the RTCM message queue this time
                pass

            time.sleep(0.05)

    def stop(self):
        self.stop_event.set()


def stop_threads(workers):
    for worker in workers:
        worker.stop()
        worker.join()


def start_threads(caster_ip, caster_port, mountpoint, username, password, broadcast_ip, broadcast_port, roslogger):
    workers = [NtripSocketThread(caster_ip, caster_port, mountpoint, username, password, roslogger), ReceiverThread(broadcast_ip, broadcast_port, roslogger)]

    for worker in workers:
        worker.start()
    return workers


class RosInterface(Node):
    def __init__(self):
        super().__init__('ntrip_forwarding')

        self.caster_ip = self.declare_parameter('caster_ip', '').value
        self.caster_port = self.declare_parameter('caster_port', 0).value
        self.mountpoint = self.declare_parameter('mountpoint', '').value
        self.username = self.declare_parameter('ntrip_username', '').value
        self.password = self.declare_parameter('ntrip_password', '').value
        self.broadcast_ip = self.declare_parameter('broadcast_ip', '195.0.0.255').value
        self.broadcast_port = self.declare_parameter('broadcast_port', 50472).value

        self.gga_timer = self.create_timer(5.0, self.gga_timer_cb)

        self.sub_nmea = self.create_subscription(String, 'nmea', self.recv_gga, 1)

        self.gga_msg = ''
        self.workers = start_threads(self.caster_ip, self.caster_port, self.mountpoint, self.username, self.password, self.broadcast_ip, self.broadcast_port, self.get_logger())

    def recv_gga(self, msg):
        self.gga_msg = msg.data

    def gga_timer_cb(self):
        if len(self.gga_msg) > 0:
            gga_queue.put(self.gga_msg)

    def on_shutdown(self):
        self.roslogger.info('Shutting down')
        stop_threads(self.workers)


if __name__ == '__main__':
    rclpy.init()
    ros_interface = RosInterface()
    rclpy.spin(ros_interface)
    ros_interface.on_shutdown()
