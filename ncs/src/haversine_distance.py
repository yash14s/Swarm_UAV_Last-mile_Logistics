import math

def haversine_distance(latitude1, longitude1, latitude2, longitude2):
    latitude1 = math.radians(latitude1)
    latitude2 = math.radians(latitude2)
    longitude1 = math.radians(longitude1)
    longitude2 = math.radians(longitude2)
    dlon = longitude2 - longitude1
    dlat = latitude2 - latitude1
    a = math.sin(dlat/2)**2 + math.cos(latitude1) * math.cos(latitude2) * math.sin(dlon/2)**2
    c = 2 * math.asin(a**0.5)
    r = 6371000
    return c*r

lat1 = -35.3632604
lon1 = 149.1652158
lat2 = -35.3632622
lon2 = 149.1652923
lat3 = -35.3632353
lon3 = 149.1652595
lat4 = -35.3632081
lon4 = 149.1652155

print(haversine_distance(lat1, lon1, lat2, lon2))
print(haversine_distance(lat1, lon1, lat3, lon3))
print(haversine_distance(lat1, lon1, lat4, lon4))
print(haversine_distance(lat2, lon2, lat3, lon3))
print(haversine_distance(lat2, lon2, lat4, lon4))
print(haversine_distance(lat3, lon3, lat4, lon4))