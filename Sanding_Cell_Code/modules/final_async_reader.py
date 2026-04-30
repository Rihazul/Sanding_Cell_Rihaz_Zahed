import minimalmodbus
import time
from modules.getPoints import changeCoordinateFinder

def scale_value(value):
    max_val = 1.8916127969920706e-40
    min_val = 9.108440018111311e-41
    # Ensure the value is within the range of min_val and max_val
    if value < min_val:
        value = min_val
    elif value > max_val:
        value = max_val

    # Scale the value to be between 65 and 135
    scaled_value = 65 + ((value - min_val) / (max_val - min_val)) * (134.99 - 65)
    return scaled_value

def init_sensor():
    PORT='COM5'
    #Set up instrument
    instrument = minimalmodbus.Instrument(PORT,1,mode=minimalmodbus.MODE_RTU, debug=False)

    #Make the settings explicit
    instrument.serial.baudrate = 115200        # Baud
    instrument.serial.bytesize = 8
    instrument.serial.parity   = minimalmodbus.serial.PARITY_NONE
    instrument.serial.stopbits = 1
    instrument.serial.timeout  = 0.1          # seconds

    # Keep the port open during a scan for performance.
    instrument.close_port_after_each_call = False
    # For repeated scans, stale bytes can cause checksum/CRC parse failures.
    # Clearing buffers per transaction is slower but significantly more stable.
    instrument.clear_buffers_before_each_transaction = True
    return instrument

def getInstrument():
    return init_sensor()

def getRawHeight(instrument):
    DIST_REGISTER = 0
    val = instrument.read_float(registeraddress=DIST_REGISTER, functioncode=4, number_of_registers=2, byteorder= minimalmodbus.BYTEORDER_BIG)
    return val

def getLaserHeight():
    DIST_REGISTER = 0
    instrument = init_sensor()
    val = instrument.read_float(registeraddress=DIST_REGISTER, functioncode=4, number_of_registers=2, byteorder= minimalmodbus.BYTEORDER_BIG)
    current_sensor_value = scale_value(val)
    # print(f'The height is: {current_value:.2f} mm', end='\r')
    # time.sleep(0.05)
    return current_sensor_value

if __name__ == "__main__":
    while (1):
        print(f"Laser Height: {getLaserHeight()} mm                                              ", end='\r')
