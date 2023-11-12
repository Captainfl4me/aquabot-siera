import math
# Cette foncion, à partir de coordonnées GPS, va les convertir en coordonnées cartésiennes 
# Input : latitude, longitude et altitude données par le GPS du bateau (en mètres)
# Output : Retourne les trois composantes cartésiennes associées (en mètres)

def geodetic_to_cartesian(latitude, longitude, altitude):
    # Rayon de la Terre en mètre (on considera que la Terre est une sphere parfaite)
    rayon_terre = 6371000.0

    # Conversion de degré à radian 
    latitude_rad = math.radians(latitude)
    longitude_rad = math.radians(longitude)

    # Calcul des coordonnées cartésiennes en mètre 
    x = (rayon_terre + altitude) * math.cos(latitude_rad) * math.cos(longitude_rad)
    y = (rayon_terre + altitude) * math.cos(latitude_rad) * math.sin(longitude_rad)
    z = (rayon_terre + altitude) * math.sin(latitude_rad)

    return x, y, z

# TEST 
latitude = 37.7749  # Exemple latitude pour San Francisco
longitude = -122.4194  # Exemple longitude pour San Francisco
altitude = 0.0  # Exemple altitude

cartesian_coords = geodetic_to_cartesian(latitude, longitude, altitude)
print("Cartesian Coordinates:", cartesian_coords)
