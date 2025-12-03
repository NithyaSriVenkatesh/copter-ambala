from PIL import Image, ImageDraw, ImageFont
import io
import math

DISCLAIMER_WIDTH = 345
DISCLAIMER_HEIGHT = 20

_EARTHPIX = 268435456  # Number of pixels in half the earth's circumference at zoom = 21
_DEGREE_PRECISION = 6  # Number of decimal places for rounding coordinates
_TILESIZE = 640        # Larget tile we can grab without paying
_GRABRATE = 10          # Fastest rate at which we can download tiles without paying


_pixrad = _EARTHPIX / math.pi

def prepareInitialImage(arrByte,width , height):
    '''
    Prepare the image recieved from PNOA to be showed in the app
    :param arrByte: array of bytes of the image
    :return: URI of the saved image
    '''
    image = Image.open(io.BytesIO(arrByte))
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype("fonts/OpenSans-Bold.ttf", 11)
    draw.text((width-DISCLAIMER_WIDTH, height-DISCLAIMER_HEIGHT), "muthu", font=font, fill="white")
    imgByteArr = io.BytesIO()
    image.save(imgByteArr, format='PNG')
    imgByteArr = imgByteArr.getvalue()
    image.save("images/imageWithDisclaimer.png")
    return imgByteArr

# DEPRECATED for CPU consumption
def refreshPosition(arrByte, x, y):
    
    image = Image.open(io.BytesIO(arrByte))
    draw = ImageDraw.Draw(image)
    draw.point([x,y],fill=(0,0,255))
    imgByteArr = io.BytesIO()
    image.save(imgByteArr, format='PNG')
    imgByteArr = imgByteArr.getvalue()
    return imgByteArr

# DEPRECATED for CPU consumption
def addWayPointImg(arrByte, x, y, count = 0):
    image = Image.open(io.BytesIO(arrByte))
    draw = ImageDraw.Draw(image)
    draw.line((x-10, y, x+10, y), fill=(255,0,0), width=2)
    draw.line((x, y-10, x, y+10), fill=(255,0,0), width=2)
    font = ImageFont.truetype("fonts/OpenSans-Bold.ttf", 11)
    draw.text((x+5, y-10), str(count))
    imgByteArr = io.BytesIO()
    image.save(imgByteArr, format='PNG')
    imgByteArr = imgByteArr.getvalue()
    return imgByteArr

def plot_lat_lon(lat, lon, center):
    offset_lat_degrees = lat - center[0]
    offset_lon_degrees = lon - center[1]
    degrees_in_map1 = (1024 / 256.0) * (360.0 / pow(2, ZOOM))
    degrees_in_map2 = (1024 / 256.0) * (360.0 / pow(2, ZOOM))
    grid_x = (offset_lon_degrees / degrees_in_map2) * 1024
    grid_y = (offset_lat_degrees / degrees_in_map1) * 1024
    x, y = grid_x + ((1024 * SCALE) / 2), grid_y + ((1024 * SCALE) / 2)
    return int(x), int(y)

def xy2deg(xy, zoom, isTile=True):
        """xy=(x,y)"""
        n = 2.0 ** zoom if isTile else 2.0 ** zoom * 256.0
        lon_deg = xy[0] / n * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * xy[1] / n)))
        lat_deg = math.degrees(lat_rad)
        return (lat_deg, lon_deg) 

def get_lat_lon(x, y, map_zoom, map_center, map_width, map_height):
	grid_x, grid_y = x - (map_width / 2), -1 * (y - (map_height / 2))
	degrees_in_map1 = (map_width / 256.0) * (360.0 / pow(2, map_zoom))
	degrees_in_map2 = (map_height / 256.0) * (360.0 / pow(2, map_zoom))
	offset_x_degrees = (float(grid_x) / map_width) * degrees_in_map1
	offset_y_degrees = (float(grid_y) / map_height) * degrees_in_map2
	return map_center[0] + offset_x_degrees, map_center[1] + offset_y_degrees

def _pixels_to_degrees(pixels, zoom):
	return pixels * 2 ** (21 - zoom)

def _pix_to_lon(j, lonpix, ntiles, _TILESIZE, zoom):
	return math.degrees((lonpix + _pixels_to_degrees(((j)-ntiles/2)*_TILESIZE, zoom) - _EARTHPIX) / _pixrad)

def _pix_to_lat(k, latpix, ntiles, _TILESIZE, zoom):
	return math.degrees(math.pi/2 - 2 * math.atan(math.exp(((latpix + _pixels_to_degrees((k-ntiles/2)*_TILESIZE, zoom)) - _EARTHPIX) / _pixrad)))


def _local_tile_step(zoomlevel, _centerLat, _centerLon, ntiles=3):   # the output stands for: at current lat & lon & zoom level, 640 pixels equivalent to how many degree of lat/lon; or the tile has how many degree of lat/lon
	latitude = _centerLat
	longitude = _centerLon

	lonpix = _EARTHPIX + longitude * math.radians(_pixrad)
	sinlat = math.sin(math.radians(latitude))
	latpix = _EARTHPIX - _pixrad * math.log((1 + sinlat)/(1 - sinlat)) / 2

	_latStep = abs(_pix_to_lat(0, latpix, ntiles, _TILESIZE, zoomlevel) - _pix_to_lat(2, latpix, ntiles, _TILESIZE, zoomlevel)) /2
	_lonStep = abs(_pix_to_lon(0, lonpix, ntiles, _TILESIZE, zoomlevel) - _pix_to_lon(2, lonpix, ntiles, _TILESIZE, zoomlevel)) /2
	return _latStep, _lonStep    

def PostoGPS(x, y, zoomlevel, _centerLat, _centerLon, width_center, height_center):
        latStep, lonStep = _local_tile_step(zoomlevel, _centerLat, _centerLon)
        local_x_to_lon = (x-width_center)/(_TILESIZE*1.0) * lonStep
        local_y_to_lat = (y-height_center)/(_TILESIZE*1.0) * latStep
        point_lon = _centerLon + local_x_to_lon
        point_lat = _centerLat - local_y_to_lat
        return point_lat,point_lon

def GPStoImagePos(tempLat, tempLon, zoomlevel, _centerLat, _centerLon, width_center, height_center):
	latStep, lonStep = _local_tile_step(zoomlevel, _centerLat, _centerLon)
	temp_point_x = ((tempLon - _centerLon)/lonStep*(1.0*_TILESIZE) + width_center)
	temp_point_y = (height_center - (tempLat - _centerLat)/latStep*(1.0*_TILESIZE))
	point_x = int(temp_point_x)
	point_y = int(temp_point_y)
	if (temp_point_x - int(temp_point_x)) >= 0.5:
	    point_x = int(temp_point_x + 1)
	if (temp_point_y - int(temp_point_y)) >= 0.5:
	    point_y = int(temp_point_y + 1)
	return point_x,point_y



def posImage2Coords(x, y, tamImageX, tamImageY, latMin, lonMin, latMax, lonMax):
    '''
    Obtains the coords referenced at a position over the image
    :param x: image x position
    :param y: image y position
    :param tamImageX: width of the image
    :param tamImageY: height of the image
    :param latMin: latitude on the y axis at the bottom left corner
    :param lonMin: longitude on the x axis at the bottom left corner
    :param latMax: latitude on the y axis at the upper right corner
    :param lonMax: longitude on the x axis at the upper right corner
    :return: lat, lon who correspond to the point in the image
    '''

    distCoordX = round(latMax - latMin,10)
    distCoordY = round(lonMax - lonMin, 10)
    '''
    lat = latMax - (x * (distCoordX/tamImageX))
    #print("lat(" + str(lat) + ") =" + str(latMax) + " - (" + str(y) + " *  (" +str(distCoordY) + "/" + str(tamImageY) + "))" )
    lon = lonMin + (y * (distCoordY/tamImageY))
    '''
    lat = latMin + ((tamImageY-y )* (distCoordY/tamImageY))
    #print("lat(" + str(lat) + ") =" + str(latMax) + " - (" + str(y) + " *  (" +str(distCoordY) + "/" + str(tamImageY) + "))" )
    lon = lonMax - ((tamImageX-x) * (distCoordX/tamImageX))
    return round(lat, 10), round(lon, 10)




##def posCoords2Image(lonMin, latMin, lonMax, latMax, lat, lon, tamImageX, tamImageY):
def posCoords2Image(lonMin, latMin, lonMax, latMax, lat, lon, tamImageX, tamImageY):
    '''
    Obtains position in the image the coords referenced for
    :param latMin: latitude on the y axis at the bottom left corner
    :param lonMin: longitude on the x axis at the bottom left corner
    :param latMax: latitude on the y axis at the upper right corner
    :param lonMax: longitude on the x axis at the upper right corner
    :param lat: latitude to covert
    :param lon: longitude to convert
    :return: x, y who correspond to the point in the image

    '''

    distCoordX = round(latMax - latMin,7)
    distCoordY = round(lonMax - lonMin, 7)
    x = ((latMax-lat) *(tamImageX))/distCoordX
    y = ((lon-lonMin) *(tamImageY))/distCoordY
    #print (x,y)
    return round(y), round(x)

