import utm

def dms2dd(degrees, minutes, seconds, direction):
    dd = float(degrees) + float(minutes)/60 + float(seconds)/(60*60);
    if direction == 'W' or direction == 'S':
        dd *= -1
    return dd;

def dd2dms(deg):
    d = int(deg)
    md = abs(deg - d) * 60
    m = int(md)
    sd = (md - m) * 60
    return [d, m, sd]

def parse_dms(dms):
    parts = dms.split(' ')
    lat = dms2dd(parts[0], parts[1], parts[2], parts[3])

    return (lat)

def dd2utm(lat, lon):
    return utm.from_latlon(lat, lon)

lat = parse_dms("38 22 20.3 N")
lon = parse_dms("110 42 16.2 W")

utmCoor = dd2utm(lat, lon)


print(lat)
print(lon)
print(utmCoor)