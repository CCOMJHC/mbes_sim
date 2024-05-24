#!/usr/bin/env python3

from osgeo import gdal
import rospy
from std_msgs.msg import Float32, Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import math
import project11
from rospy.timer import TimerEvent


class BathyGrid:
    def __init__(self, fname):
        #rospy.loginfo("grid file: "+fname)
        self.dataset = gdal.Open(fname, gdal.GA_ReadOnly)
        #print ('opened',self.dataset.GetDescription())
        self.band = self.dataset.GetRasterBand(1)
        self.geoTransform = self.dataset.GetGeoTransform()
        self.inverseGeoTransform = gdal.InvGeoTransform(self.geoTransform)
        #print (self.geoTransform)
        #print ('inverse geoTransform', self.inverseGeoTransform)
        self.data = self.band.ReadAsArray()
        #print (self.data.shape)
        
        sourceSR = gdal.osr.SpatialReference()
        sourceSR.SetWellKnownGeogCS("WGS84")
        
        targetSR = gdal.osr.SpatialReference()
        targetSR.ImportFromWkt(self.dataset.GetProjection())
        
        self.coordinateTransformation = gdal.osr.CoordinateTransformation(sourceSR, targetSR)

    def getXY(self,lat,lon):
        return self.coordinateTransformation.TransformPoint(lat,lon)[:2]
        
    def getDepthAtLatLon(self,lat,lon):
        x,y = self.getXY(lat,lon)
        #print ('lat,lon:',lat,lon,'x,y',x,y)
        return self.getDepth(x,y)
    
    def getDepth(self,x,y):
        xi = self.inverseGeoTransform[0]+x*self.inverseGeoTransform[1]+y*self.inverseGeoTransform[2]
        yi = self.inverseGeoTransform[3]+x*self.inverseGeoTransform[4]+y*self.inverseGeoTransform[5]
        #print ('index:',xi,yi)
        try:
            return self.data[int(yi),int(xi)]
        except IndexError:
            return None


class SonarSim:
    def __init__(self, bathy: BathyGrid):
        self.bathy = bathy
        self.robot = project11.nav.RobotNavigation()

        self.swath_angle = rospy.get_param('~swath_angle', 120.0)
        self.beam_count = rospy.get_param('~beam_count', 20)
        self.ping_rate = rospy.get_param('~ping_interval', 1.0)

        self.tan_half_swath_angle = math.tan(math.radians(self.swath_angle/2.0))

        self.frame_id = rospy.get_param('~sonar_frame_id', 'mbes')

        self.depth_publisher = rospy.Publisher('depth', Float32, queue_size = 5)
        self.ping_publisher = rospy.Publisher('mbes_ping', PointCloud2, queue_size=10)

        self.ping_timer =  rospy.Timer(rospy.Duration(self.ping_rate), self.ping_callback)

    def ping_callback(self, event: TimerEvent):
        position = self.robot.positionLatLon()
        lon_rad =position[1]
        lat_rad = position[0]
        lon_deg = math.degrees(lon_rad)
        lat_deg = math.degrees(lat_rad)

        depth = self.bathy.getDepthAtLatLon(lat_deg, lon_deg)
        #rospy.loginfo("depth: " + str(depth)) #print depth

        if depth is not None and depth >= 0:
            self.depth_publisher.publish(depth)
            heading = self.robot.heading()
            if heading is not None:
                swath_half_width = depth*self.tan_half_swath_angle
                #print 'swath half width:',swath_half_width
                port_outer_beam_location = project11.geodesic.direct(lon_rad, lat_rad, math.radians(heading-90),swath_half_width)
                starboard_outer_beam_location = project11.geodesic.direct(lon_rad, lat_rad, math.radians(heading+90),swath_half_width)
                #print 'outer beam locations:',port_outer_beam_location,starboard_outer_beam_location
                port_outer_beam_location_xy = grid.getXY(math.degrees(port_outer_beam_location[1]), math.degrees(port_outer_beam_location[0]))
                starboard_outer_beam_location_xy = grid.getXY(math.degrees(starboard_outer_beam_location[1]), math.degrees(starboard_outer_beam_location[0]))
                dx = (starboard_outer_beam_location_xy[0] - port_outer_beam_location_xy[0])/float(self.beam_count)
                dy = (starboard_outer_beam_location_xy[1] - port_outer_beam_location_xy[1])/float(self.beam_count)
                
                sounding_spacing = 2.0*swath_half_width/float(self.beam_count)
                
                soundings = []

                for i in range(self.beam_count):
                    x = port_outer_beam_location_xy[0] + dx*i
                    y = port_outer_beam_location_xy[1] + dy*i
                    z = grid.getDepth(x,y)
                    #print 'depth:', z
                    if z is not None:
                        soundings.append((0.0, -swath_half_width+i*sounding_spacing, z))

                fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1)
                ]
                header = Header()
                header.frame_id = self.frame_id
                header.stamp = event.current_real

                pc2 = point_cloud2.create_cloud(header, fields, soundings)

                self.ping_publisher.publish(pc2)
                        


if __name__ == '__main__':
    rospy.init_node('mbes_sim')

    grid_file = rospy.get_param('~grid_file')
    grid = BathyGrid(grid_file)

    sonar = SonarSim(grid)
    
    rospy.spin()
