import math
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 14550

DEG2RAD = math.pi/180
RAD2DEG = 180/math.pi

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

def degmin_to_degdec (num):
    deg_whole = float(int(num / 100))
    minutes = num % 100
    return deg_whole + minutes / 60


def calc_midpoint (lat1, lon1, lat2, lon2):
    dlon = lon2 - lon1
    clat1 = math.cos(lat1)
    slat1 = math.sin(lat1)
    clat2 = math.cos(lat2)
    slat2 = math.sin(lat2)
    Bx = clat2 * math.cos(dlon)
    By = clat2 * math.sin(dlon)
    lat_mid = math.atan2(slat1 + slat2, math.sqrt((clat1 + Bx)**2 + By**2))
    lon_mid = lon1 + math.atan2(By,clat1 + Bx)
    return [lat_mid * RAD2DEG, lon_mid * RAD2DEG]

while True:
    data, addr = sock.recvfrom(2048)
    raw_str = data.decode('utf-8')
    str_splits = raw_str.split(',')

    if (str_splits[0] != "$GPGGA"):
        continue

    #print (raw_str)
    lat_num_str = str_splits[2]
    lat_dir_str = str_splits[3]
    lon_num_str = str_splits[4]
    lon_dir_str = str_splits[5]
    raw_lat_num = float(lat_num_str)
    raw_lon_num = float(lon_num_str)
    lat_deg = degmin_to_degdec(raw_lat_num)
    if (lat_dir_str == "S"):
        lat_deg *= -1
    lon_deg = degmin_to_degdec(raw_lon_num)
    if (lat_dir_str == "W"):
        lon_deg *= -1
    print (lon_deg)
