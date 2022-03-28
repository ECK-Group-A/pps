import time
import socket
import sched

UDP_IP = '1.1.1.1'
UDP_PORT = 10110

def calc_checksum(sequence):
    calc_cksum = 0

    for s in nmeadata.split('*')[0]:
        if s in ['$', ',']: 
            continue
        calc_cksum ^= ord(s)
    
    return calc_cksum

last_time = 0

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    while True:
        if last_time != round(time.time()):
            last_time = round(time.time())
            t = time.localtime()
            time.sleep(0.05)

            nmeadata = ','.join(['$GPRMC', # Recommended Minimum sentence
                                time.strftime("%H%M%S", t), # Time fix is taken
                                'A', # Receiver status: A = Active, V = Void
                                '0000.000,N', # Latitude N
                                '00000.000,E', # Longitude E
                                '000.0', # Speed over the ground (knots)
                                '000.0', # Track made good (degrees True)
                                time.strftime("%d%m%y", t), # Date fix is taken
                                '000.0,W', # MagneticVariation
                                'S*']) # Fix Mode Indicator: (A)utonomous, (D)ifferential, (E)stimated, (N)ot valid, (S)imulator.

            nmeaseq =  nmeadata + format(calc_checksum(nmeadata), 'x')

            sock.sendto(bytes(nmeaseq, "utf-8"), (UDP_IP, UDP_PORT))
            print("Send:", nmeaseq)