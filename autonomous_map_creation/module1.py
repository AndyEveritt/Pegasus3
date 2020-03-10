
import gdal
import sys
import os
# this allows GDAL to throw Python Exceptions
gdal.UseExceptions()

file='./Utah/Aerial/q3128_se_NAIP2016_RGB.tif'
try:
    gtif  = gdal.Open( file )
except RuntimeError, e:
    print 'Unable to open '+file
    print e
    sys.exit(1)

#print gtif.GetMetadata()
statinfo = os.stat(file)
size=statinfo.st_size/1e6

print size,"Mb"

print "bands ",gtif.RasterCount

cols = gtif.RasterXSize
rows = gtif.RasterYSize
print cols," cols x ",rows," rows"
gtif = None