#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import threading
import struct
import math
from cobs import cobs
from crccheck.crc import Crc32Mpeg2

#  ROS 2
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
# Protobuf generated classes
import rx_pb2
import tx_pb2

class Stm32SerialBridge(Node):
    def __init__(self):
        super().__init__('stm32_serial_bridge')

        # --- 1.(Parameters) ---
        self.declare_parameter('serial_port', '/dev/serial0')  
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value

        # --- 2. (Serial) ---
        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"✅ Connected to STM32 on {port} at {baud} bps.")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed to connect to serial port: {e}")
            raise SystemExit

        self.serial_lock = threading.Lock() 

        # --- 3.ROS 2 Publishers & Subscribers ---
        #  Nav2
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        #  Robot Localization
        self.odom_pub = self.create_publisher(Odometry, 'odom_unfiltered', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # --- 4.  (Read Thread) ---
        self.running = True
        self.rx_thread = threading.Thread(target=self.serial_rx_loop, daemon=True)
        self.rx_thread.start()

    # ==========================================
    #           From ROS 2 to STM32
    # ==========================================
    def cmd_vel_callback(self, msg: Twist):
        try:
            # 1. Protobuf Serialization
            rx_msg = rx_pb2.RxMsg()
            rx_msg.CmdVel.linearX = msg.linear.x
            rx_msg.CmdVel.AngleZ = msg.angular.z
            
            proto_data = rx_msg.SerializeToString()

            # 2. Padding STM32 Hardware CRC
            remainder = len(proto_data) % 4
            padded_data = proto_data
            if remainder > 0:
                padded_data += b'\x00' * (4 - remainder)

            stm32_ordered_data = bytearray()
            for i in range(0, len(padded_data), 4):
                word = padded_data[i:i+4]
                stm32_ordered_data.extend(word[::-1])

            calc_crc = Crc32Mpeg2.calc(stm32_ordered_data)

            # 3.payload + CRC
            payload_with_crc = proto_data + struct.pack('<I', calc_crc)

            # 4.  COBS encodeing + Frame Delimiter
            encoded_data = cobs.encode(payload_with_crc) + b'\x00'

            # 5.  Send to STM32
            with self.serial_lock:
                self.serial_port.write(encoded_data)
                
        except Exception as e:
            self.get_logger().error(f"❌ Error sending cmd_vel to STM32: {e}")

    # ==========================================
    #          From STM32 to ROS 2
    # ==========================================
    def serial_rx_loop(self):
        buffer = bytearray()
        while self.running and rclpy.ok():
            try:
                if self.serial_port.in_waiting > 0:
                    chunk = self.serial_port.read(self.serial_port.in_waiting)
                    buffer.extend(chunk)

                    # 0x00 COBs frame delimiter 
                    while b'\x00' in buffer:
                        frame_end = buffer.index(b'\x00')
                        frame = buffer[:frame_end]
                        buffer = buffer[frame_end + 1:]

                        if len(frame) > 0:
                            self.process_incoming_frame(frame)
            except Exception as e:
                self.get_logger().error(f"⚠️ Serial read error: {e}")
                # Optional: Add reconnection logic here

    def process_incoming_frame(self, frame):
        try:
            # 1.  COBS decodeing
            decoded_data = cobs.decode(frame)

            if len(decoded_data) < 4:
                return

            # 2. CRC separation
            data_part = decoded_data[:-4]
            received_crc = struct.unpack('<I', decoded_data[-4:])[0]

            # 3. Padding STM32 Hardware CRC for calculation
            remainder = len(data_part) % 4
            padded_data = data_part
            if remainder > 0:
                padded_data += b'\x00' * (4 - remainder)

            stm32_ordered_data = bytearray()
            for i in range(0, len(padded_data), 4):
                word = padded_data[i:i+4]
                stm32_ordered_data.extend(word[::-1])

            calc_crc = Crc32Mpeg2.calc(stm32_ordered_data)

            if calc_crc != received_crc:
                self.get_logger().warn(f"❌ CRC Mismatch! Calc: {hex(calc_crc)} | Recv: {hex(received_crc)}")
                return

            # 4. Protobuf deserialization
            tx_msg = tx_pb2.TxMsg()
            tx_msg.ParseFromString(data_part)

            # 5. Publish to ROS 2
            odom_current_time = tx_msg.Odom.odomTimeStamp_us / 1_000_000.0  # Convert microseconds to seconds
            imu_current_time = tx_msg.Imu.mpu6500TimeStamp_us / 1_000_000.0

            if tx_msg.HasField("odom"):
                self.publish_odom(tx_msg.odom)
                
            if tx_msg.HasField("imu"):
                self.publish_imu(tx_msg.imu)

        except cobs.DecodeError:
            self.get_logger().warn("⚠️ COBS Decode Error: Frame corrupted.")
        except Exception as e:
            self.get_logger().error(f"❌ Frame processing error: {e}")

    # ==========================================
    #          Helper Functions for ROS 2 Messages
    # ==========================================
    def yaw_to_quaternion(self, yaw) -> Quaternion:
        """تحويل زاوية الانعراج (Yaw) إلى نظام الكواتيرنيون لـ ROS 2"""
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q

    def publish_odom(self, stm_odom):
        odom_msg = Odometry()
        odom_msg.header.stamp = stm_odom.odomTimeStamp_us / 1_000_000.0  # Convert microseconds to seconds
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        #  (Position)
        odom_msg.pose.pose.position.x = stm_odom.x
        odom_msg.pose.pose.position.y = stm_odom.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation =stm_odom.yaw


        #  (Velocity)
        odom_msg.twist.twist.linear.x = stm_odom.linear_x
        odom_msg.twist.twist.angular.z = stm_odom.angular_z

        # (Covariance matrix) EKF

        self.odom_pub.publish(odom_msg)

    def publish_imu(self, stm_imu):
        imu_msg = Imu()
        imu_msg.header.stamp = stm_imu.mpu6500TimeStamp_us / 1_000_000.0  # Convert microseconds to seconds
        imu_msg.header.frame_id = "imu_link"

        # (Linear Acceleration) - 
        imu_msg.linear_acceleration.x = stm_imu.ax
        imu_msg.linear_acceleration.y = stm_imu.ay
        imu_msg.linear_acceleration.z = stm_imu.az

        # (Angular Velocity)
        imu_msg.angular_velocity.x = stm_imu.gx
        imu_msg.angular_velocity.y = stm_imu.gy
        imu_msg.angular_velocity.z = stm_imu.gz

        self.imu_pub.publish(imu_msg)

    def destroy_node(self):
        self.running = False
        if self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bridge_node = Stm32SerialBridge()
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        bridge_node.get_logger().info("🛑 Shutting down serial bridge node.")
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
