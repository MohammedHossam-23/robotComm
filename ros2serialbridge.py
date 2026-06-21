#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import threading
import struct
import math
from cobs import cobs
from crccheck.crc import Crc32Mpeg2
from builtin_interfaces.msg import Time

# ROS 2 Messages
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# Protobuf generated classes
import rx_pb2
import tx_pb2

class Stm32SerialBridge(Node):
    def __init__(self):
        super().__init__('stm32_serial_bridge')

        # --- 1. Parameters ---
        self.declare_parameter('serial_port', '/dev/serial0')  # Update this if needed (e.g., 'COM12')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value

        # --- 2. Serial Initialization ---
        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"✅ Connected to STM32 on {port} at {baud} bps.")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed to connect to serial port: {e}")
            raise SystemExit

        self.serial_lock = threading.Lock() 

        # --- 3. ROS 2 Publishers & Subscribers ---
        # Subscriber for Navigation (Sending to STM32)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishers for Robot Localization (Receiving from STM32)
        self.odom_pub = self.create_publisher(Odometry, 'odom_unfiltered', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # --- 4. Read Thread ---
        self.running = True
        self.rx_thread = threading.Thread(target=self.serial_rx_loop, daemon=True)
        self.rx_thread.start()

    # ==========================================
    #            From ROS 2 to STM32
    # ==========================================
    def cmd_vel_callback(self, msg: Twist):
        try:
            # 1. Protobuf Serialization
            rx_msg = rx_pb2.RxMsg()
            rx_msg.cmd_vel.linear_x = msg.linear.x
            rx_msg.cmd_vel.angular_z = msg.angular.z
            
            proto_data = rx_msg.SerializeToString()

            # 2. Padding to ensure we have full 32-bit words
            remainder = len(proto_data) % 4
            padded_data = proto_data
            if remainder > 0:
                padded_data += b'\x00' * (4 - remainder)

            # 3. Re-order bytes to match STM32 Hardware casting
            stm32_ordered_data = bytearray()
            for i in range(0, len(padded_data), 4):
                word = padded_data[i:i+4]
                # Flip the word: [0,1,2,3] -> [3,2,1,0]
                stm32_ordered_data.extend(word[::-1])

            # 4. Calculate CRC on the re-ordered data
            calc_crc = Crc32Mpeg2.calc(stm32_ordered_data)

            # 5. Payload + CRC
            payload_with_crc = proto_data + struct.pack('<I', calc_crc)

            # 6. COBS encoding + Frame Delimiter (Explicitly cast to bytes)
            encoded_data = cobs.encode(bytes(payload_with_crc)) + b'\x00'

            # 7. Send to STM32
            with self.serial_lock:
                self.serial_port.write(encoded_data)
                
        except Exception as e:
            self.get_logger().error(f"❌ Error sending cmd_vel to STM32: {e}")

    # ==========================================
    #            From STM32 to ROS 2
    # ==========================================
    def serial_rx_loop(self):
        buffer = bytearray()
        while self.running and rclpy.ok():
            try:
                if self.serial_port.in_waiting > 0:
                    chunk = self.serial_port.read(self.serial_port.in_waiting)
                    buffer.extend(chunk)

                    # 0x00 COBS frame delimiter 
                    while b'\x00' in buffer:
                        frame_end = buffer.index(b'\x00')
                        frame = buffer[:frame_end]
                        buffer = buffer[frame_end + 1:]

                        if len(frame) > 0:
                            self.process_incoming_frame(frame)
            except Exception as e:
                self.get_logger().error(f"⚠️ Serial read error: {e}")

    def process_incoming_frame(self, frame):
        try:
            # 1. COBS decoding (Explicit conversion to bytes is safer for the cobs library)
            decoded_data = cobs.decode(bytes(frame))

            if len(decoded_data) < 4:
                return

            # 2. CRC separation
            data_part = decoded_data[:-4]
            received_crc = struct.unpack('<I', decoded_data[-4:])[0]

            # 3. Padding to ensure we have full 32-bit words
            remainder = len(data_part) % 4
            padded_data = data_part
            if remainder > 0:
                padded_data += b'\x00' * (4 - remainder)

            # 4. THE FIX: Re-order bytes to match STM32 Hardware casting
            stm32_ordered_data = bytearray()
            for i in range(0, len(padded_data), 4):
                word = padded_data[i:i+4]
                # Flip the word: [0,1,2,3] -> [3,2,1,0]
                stm32_ordered_data.extend(word[::-1])

            # 5. Calculate CRC on the re-ordered data
            calc_crc = Crc32Mpeg2.calc(stm32_ordered_data)

            if calc_crc != received_crc:
                self.get_logger().warn(f"❌ CRC Mismatch! Calc: {hex(calc_crc)} | Recv: {hex(received_crc)}")
                return

            # 6. Protobuf deserialization
            tx_msg = tx_pb2.TxMsg()
            tx_msg.ParseFromString(bytes(data_part))

            # ==========================================
            # 🖨️ PRINTING THE RECEIVED DATA
            # ==========================================
            self.get_logger().info("✅ RECEIVED NEW PROTOBUF DATA:")
            
            if tx_msg.HasField("odom"):
                self.get_logger().info(f"   📍 ODOM -> X: {tx_msg.odom.x:.3f}, Y: {tx_msg.odom.y:.3f}, Yaw: {tx_msg.odom.yaw:.3f}")
                
            if tx_msg.HasField("imu"):
                self.get_logger().info(f"   🚀 IMU  -> Ax: {tx_msg.imu.ax:.3f}, Gz: {tx_msg.imu.gz:.3f}")
                
            if tx_msg.HasField("battery"):
                self.get_logger().info(f"   🔋 BATT -> SoC: {tx_msg.battery.StateOfCharge}%, Status: {tx_msg.battery.ChargingState}")
                
            if tx_msg.HasField("tofArray"):
                self.get_logger().info(f"   📏 TOF  -> C1: {tx_msg.tofArray.cliff1}, C2: {tx_msg.tofArray.cliff2}")
                
            self.get_logger().info("-" * 40)
            # ==========================================

            # 7. Publish to ROS 2
            if tx_msg.HasField("odom"):
                self.publish_odom(tx_msg.odom)
                
            if tx_msg.HasField("imu"):
                self.publish_imu(tx_msg.imu)

        except cobs.DecodeError as e:
            self.get_logger().warn(f"⚠️ COBS Decode Error: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ Frame processing error: {e}")

    # ==========================================
    #          Helper Functions for ROS 2
    # ==========================================
    def yaw_to_quaternion(self, yaw) -> Quaternion:
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q

    def publish_odom(self, stm_odom):
        odom_msg = Odometry()
        
        # Convert microseconds to ROS 2 Time components
        total_seconds = stm_odom.odom_time_stamp_us / 1000000.0
        sec = int(total_seconds)
        nanosec = int((total_seconds - sec) * 1e9)
        odom_msg.header.stamp = Time(sec=sec, nanosec=nanosec)
        
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Position
        odom_msg.pose.pose.position.x = stm_odom.x
        odom_msg.pose.pose.position.y = stm_odom.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation
        odom_msg.pose.pose.orientation = self.yaw_to_quaternion(stm_odom.yaw)

        # Velocity
        odom_msg.twist.twist.linear.x = stm_odom.linear_x
        odom_msg.twist.twist.angular.z = stm_odom.angular_z

        self.odom_pub.publish(odom_msg)

    def publish_imu(self, stm_imu):
        imu_msg = Imu()
        
        # Convert microseconds to ROS 2 Time components
        total_seconds = stm_imu.mpu6500_time_stamp_us / 1000000.0
        sec = int(total_seconds)
        nanosec = int((total_seconds - sec) * 1e9)
        imu_msg.header.stamp = Time(sec=sec, nanosec=nanosec)
        
        imu_msg.header.frame_id = "imu_link"

        # Linear Acceleration
        imu_msg.linear_acceleration.x = stm_imu.ax
        imu_msg.linear_acceleration.y = stm_imu.ay
        imu_msg.linear_acceleration.z = stm_imu.az

        # Angular Velocity
        imu_msg.angular_velocity.x = stm_imu.gx
        imu_msg.angular_velocity.y = stm_imu.gy
        imu_msg.angular_velocity.z = stm_imu.gz

        self.imu_pub.publish(imu_msg)

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
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
