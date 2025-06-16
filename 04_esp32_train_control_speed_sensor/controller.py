import asyncio
from bleak import BleakClient, BleakScanner
import time

SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
CHAR_UUID = "87654321-4321-4321-4321-cba987654321"

class SimpleBLETest:
    def __init__(self):
        self.client = None
        self.connected = False
        
    def notification_handler(self, sender, data):
        """Handle notifications from ESP32"""
        try:
            # Try to decode as UTF-8 text first
            message = data.decode('utf-8')
            print(f"📨 ESP32 Text: {message}")
        except UnicodeDecodeError:
            # If binary data, show hex representation
            hex_data = ' '.join(f'{b:02x}' for b in data)
            print(f"📨 ESP32 Binary: {hex_data} (length: {len(data)} bytes)")
            
            # Try to interpret as float if 4 bytes
            if len(data) == 4:
                try:
                    import struct
                    float_val = struct.unpack('f', data)[0]
                    print(f"   └─ As float: {float_val}")
                except:
                    pass
        
    async def scan_and_find_esp32(self):
        """Scan for ESP32 device"""
        print("🔍 Scanning for ESP32 device...")
        
        try:
            devices = await BleakScanner.discover(timeout=10.0)
            print(f"📡 Found {len(devices)} devices:")
            
            esp32_candidates = []
            
            for device in devices:
                name = device.name if device.name else "Unknown"
                print(f"  📱 {name} ({device.address})")
                
                # Check for exact match first
                if device.name and any(exact in device.name for exact in ["ESP32-LEGO-Bridge", "ESP32-LEGO","ESP32-LE","ESP32"]):
                    print(f"✅ Found exact match: {device.address}")
                    return device.address
                
                # Check for partial matches (truncated names)
                if device.name and any(keyword in device.name.upper() for keyword in 
                    ["ESP32", "ESP32-LE", "ESP32-LEGO"]):
                    esp32_candidates.append((device.name, device.address))
                    print(f"🎯 Potential ESP32 found: {name} ({device.address})")
            
            if esp32_candidates:
                print(f"\n✅ Found {len(esp32_candidates)} ESP32-like device(s):")
                for i, (name, addr) in enumerate(esp32_candidates, 1):
                    print(f"  {i}. {name} ({addr})")
                
                if len(esp32_candidates) == 1:
                    chosen_addr = esp32_candidates[0][1]
                    print(f"🎯 Auto-selecting: {chosen_addr}")
                    return chosen_addr
                else:
                    # Multiple candidates, let user choose
                    try:
                        choice = int(input(f"\nSelect device (1-{len(esp32_candidates)}): ")) - 1
                        if 0 <= choice < len(esp32_candidates):
                            return esp32_candidates[choice][1]
                    except (ValueError, IndexError):
                        print("❌ Invalid selection")
                        
            print("❌ No ESP32 devices found")
            return None
            
        except Exception as e:
            print(f"❌ Scan failed: {e}")
            return None
    
    async def connect_to_esp32(self, address):
        """Connect to ESP32"""
        try:
            print(f"🔗 Connecting to {address}...")
            
            self.client = BleakClient(address)
            await self.client.connect()
            
            if not self.client.is_connected:
                raise Exception("Connection failed")
                
            print("✅ Connected successfully!")
            
            # Check services
            try:
                services = self.client.services
            except AttributeError:
                services = await self.client.get_services()
                
            print("📋 Available services:")
            for service in services:
                print(f"  🔧 {service.uuid}")
                
            # Start notifications
            await self.client.start_notify(CHAR_UUID, self.notification_handler)
            print("📡 Notifications enabled")
            
            self.connected = True
            return True
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    async def send_command(self, command):
        """Send command to ESP32"""
        if not self.client or not self.client.is_connected:
            print("❌ Not connected")
            return False
            
        try:
            await self.client.write_gatt_char(CHAR_UUID, command.encode())
            print(f"📤 Sent: {command}")
            return True
        except Exception as e:
            print(f"❌ Send failed: {e}")
            return False
    
    async def run_tests(self):
        """Run a series of tests"""
        print("\n🧪 Running BLE Communication Tests...")
        
        test_commands = [
            "TEST_PING",
            "GET_STATUS", 
            "SET_VALUE_42",
            "LED_RED",
            "LED_GREEN", 
            "LED_BLUE",
            "LED_BLINK",
            "GET_STATUS"
        ]
        
        for i, cmd in enumerate(test_commands, 1):
            print(f"\n--- Test {i}: {cmd} ---")
            await self.send_command(cmd)
            await asyncio.sleep(2)  # Wait for response
            
    async def interactive_mode(self):
        """Interactive command mode"""
        print("\n🎮 Interactive Mode (type 'quit' to exit)")
        print("Available commands:")
        print("  TEST_PING, GET_STATUS, SET_VALUE_<number>")
        print("  LED_RED, LED_GREEN, LED_BLUE, LED_WHITE, LED_OFF, LED_BLINK")
        
        while self.connected:
            try:
                command = input("\n💬 Enter command: ").strip()
                if command.lower() in ['quit', 'exit']:
                    break
                if command:
                    await self.send_command(command)
                    await asyncio.sleep(1)
            except KeyboardInterrupt:
                break
                
    async def disconnect(self):
        """Disconnect from ESP32"""
        if self.client and self.client.is_connected:
            try:
                await self.client.stop_notify(CHAR_UUID)
                await self.client.disconnect()
                print("🔌 Disconnected")
            except Exception as e:
                print(f"❌ Disconnect error: {e}")
        self.connected = False

async def main():
    print("🚀 ESP32 BLE Test Script")
    print("=" * 40)
    
    test = SimpleBLETest()
    
    try:
        # Step 1: Scan for device
        address = await test.scan_and_find_esp32()
        if not address:
            print("\n💡 Troubleshooting tips:")
            print("  - Make sure ESP32 is powered on")
            print("  - Check if 'ESP32-LEGO-Bridge' appears in Bluetooth settings")
            print("  - Try moving closer to the ESP32")
            return
        
        # Step 2: Connect
        if not await test.connect_to_esp32(address):
            return
            
        # Step 3: Run automatic tests
        await test.run_tests()
        
        # Step 4: Interactive mode
        await test.interactive_mode()
        
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        await test.disconnect()
        print("👋 Test completed")

if __name__ == "__main__":
    asyncio.run(main())