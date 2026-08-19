import serial

ser = serial.Serial("COM6", 9600, timeout=0.2)

print("Listening...")

while True:
        d = ser.read(200)
        if d:
            print(d)
            print("HEX:", d.hex())

