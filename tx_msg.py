import serial
import cobs
from cobs import cobs
import struct
import crccheck 
from crccheck import crc  
from crccheck.crc import Crc32Mpeg2
import tx_pb2  # Ensure tx_pb2.py is in the same directory
import rx_pb2

def process_payload(payload):
    if len(payload) < 4:
        return

    data_part = payload[:-4] #Data = [1:2:3:4:5:6:7:8:9:10:11:12:13:14:15:16:17:18:19:20] ;Data0[:-4] 
    received_crc = struct.unpack('<I', payload[-4:])[0]

    # 1. Padding to ensure we have full 32-bit words
    remainder = len(data_part) % 4
    padded_data = data_part
    if remainder > 0:
        padded_data += b'\x00' * (4 - remainder)

    # 2. THE FIX: Re-order bytes to match STM32 Hardware casting
    # We must treat the data as a series of 32-bit Little-Endian words
    # but calculate CRC as if they were Big-Endian (how the engine sees them).
    stm32_ordered_data = bytearray()
    for i in range(0, len(padded_data), 4):
        word = padded_data[i:i+4]                                         
        # Flip the word: [0,1,2,3] -> [3,2,1,0]
        stm32_ordered_data.extend(word[::-1])

    # 3. Calculate CRC on the re-ordered data
    calc_crc = Crc32Mpeg2.calc(stm32_ordered_data)

    if calc_crc != received_crc:
        print(f"❌ CRC Mismatch!")
        print(f"   Calculated: {hex(calc_crc)} | Received: {hex(received_crc)}")
        return

    print("✅ CRC OK! Parsing Protobuf...")
    # ... rest of your protobuf code ...
    # 4. Protobuf Decoding
    try:
        # Initializing the message class generated from your .proto file
        msg = tx_pb2.TxMsg() 
        msg.ParseFromString(data_part)
        
        print("✅ Valid Data Received:")
        print(f"   Accel: X={msg.imu.ax:.3f}, Y={msg.imu.ay:.3f}, Z={msg.imu.az:.3f}")
        print(f"   Gyro:  X={msg.imu.gx:.3f}, Y={msg.imu.gy:.3f}, Z={msg.imu.gz:.3f}")
        print("-" * 40)
        

        
    except Exception as e:
        print(f"❌ Protobuf Parsing Failed: {e}")

def listen_to_stm32(port='COM12', baudrate=115200):
    """
    Listens to the serial port and frames incoming COBS data.
    """
    try:
        # Initialize Serial Port
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"📡 Connected to {port}... Waiting for data...")
        
        buffer = bytearray()
        
        while True:
            if ser.in_waiting > 0:
                # Read available chunks from the OS serial buffer
                chunk = ser.read(ser.in_waiting)
                
                for byte in chunk:
                    if byte == 0x00:  # COBS Frame Delimiter found
                        if buffer:
                            try:
                                # Decode COBS
                                # Explicit conversion to bytes is safer for the cobs library
                                decoded_data = cobs.decode(bytes(buffer))
                                process_payload(decoded_data)
                            except Exception as e:
                                print(f"⚠️ COBS Decode Error: {e}")
                            
                            buffer.clear() # Clear buffer for the next frame
                    else:
                        buffer.append(byte)
                        
    except KeyboardInterrupt:
        print("\n🛑 Reception stopped by user.")
    except Exception as e:
        print(f"🚨 Serial Port Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("🔌 Serial port closed.")

if __name__ == "__main__":
    listen_to_stm32()
