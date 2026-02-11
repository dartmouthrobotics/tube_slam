#!/usr/bin/env python3
"""
Simple example script to listen for Sonar 3D-15 data over multicast and store it to a file for later.
The "inspect_sonar_data.py" script can be used to read the data from the file and decode it.

REQUIREMENTS:
  - Python 3
  - Access to the multicast IP and port used by the Sonar 3D-15
  - Internet/Firewall settings that allow UDP multicast traffic

USAGE:
  1. Run: python save_sonar_data.py
  2. Press Ctrl+C to stop the script.
"""
import socket
import struct
import rospy
from std_msgs.msg import String
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import UInt8MultiArray


# Multicast group and port used by the Sonar 3D-15
MULTICAST_GROUP = '224.0.0.96'
PORT = 4747

# Listen to all IPs by default, or set to a specific IP.
SONAR_IP = ""

# The maximum possible packet size for Sonar 3D-15 data
BUFFER_SIZE = 65535

pub_raw = None
pub_raw2 = None

def receive_multicast(filename: str):
    """
    Listen for Sonar 3D-15 UDP multicast packets on a specific port.
    - Filters packets based on the known Sonar IP address.
    - Parses the RIP1 framing.
    - Decodes the Protobuf message.
    - Prints relevant info (e.g. dimension, FoV, timestamp).
    """
    # Set up a UDP socket with multicast membership
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', PORT))

    group = socket.inet_aton(MULTICAST_GROUP)
    mreq = struct.pack('4sL', group, socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    print(f"Listening for Sonar 3D-15 RIP1 packets on {MULTICAST_GROUP}:{PORT}...")
    if SONAR_IP != "":
        print(f"Filtering packets from IP: {SONAR_IP}")

    print(f"Saving data to: {filename}")
    print("Press Ctrl+C to stop.\n")

    with open(filename, 'wb') as f:

        try:
            while not rospy.is_shutdown():
                data, addr = sock.recvfrom(BUFFER_SIZE)

                # If SONAR_IP is configured, and this doesn't match the known Sonar IP, skip it.
                if SONAR_IP != "" and addr[0] != SONAR_IP:
                    continue

                print(f"Received {len(data)} bytes from {addr}")

                
                f.write(data)
                f.flush()

                try:
                    #pub_raw.publish(data)
                    msg = UInt8MultiArray()
                    #msg = ByteMultiArray()
                    msg.data = list(data) # Convert bytes to a list of integers
                    pub_raw2.publish(msg)
                except:
                    rospy.logerr("Not working")
                    continue

        except KeyboardInterrupt:
            print("\nStopping multicast receiver.")
        finally:
            sock.close()


if __name__ == "__main__":
    rospy.init_node('3dsonar')
    pub_raw = rospy.Publisher('sonar_3d/raw_data', String, queue_size=10)
    pub_raw2 = rospy.Publisher('sonar_3d/raw_data_multibyte', UInt8MultiArray, queue_size=10)

    import argparse
    parser = argparse.ArgumentParser(
        description="Receive data from a Sonar 3D-15 via Multicast and save to file.")

    parser.add_argument(
        "--ip",
        type=str,
        default="",
        help="Limit to packets from this IP address (default: all)."
    )
    parser.add_argument(
        "--file",
        type=str,
        default="",
        help="Output filename for received data (default: )."
    )
    parser.add_argument(
        "--index",
        type=str,
        default="",
        help="Index of filename"
    )
    # Parse arguments
    args = parser.parse_args()
    if args.ip:
        SONAR_IP = args.ip
        print(f"Listening for packets from IP: {SONAR_IP}")

    print("Listening for multicast packets...")
    # Make filename from timestamp as yyyy-mm-dd-hh-mm-ss if not provided
    filename = args.file
    if filename == "":
        from datetime import datetime
        now = datetime.now()
        ts = now.strftime("%Y-%m-%d-%H-%M-%S")
        filename = f"sonar-capture-{ts}-{args.index}.sonar"

    receive_multicast(filename)
