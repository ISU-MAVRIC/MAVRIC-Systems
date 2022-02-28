import complex_globals as g
from geographiclib.geodesic import Geodesic
import math as m

class Pathing():
    def __init__(self, rover_width, avoidance_mod):
        self.w = rover_width
        self.a_mod = avoidance_mod
        self.referance_point = g.position
        self.end_point = None
        self.obstacle_num = 0
        self.enter_point = [0, 0]
        self.exit_point = [0, 0]

    # function for calculating if object intersects 
    def check_intersection(self, WP_r, WP_theta, O_r, O_theta, O_R):
        # get avoidance radius
        A_R = self.a_mod*O_R+self.w/2

        # check if object intersects with path
        i_theta = m.atan(A_r/O_r)
        delta_theta = abs(O_theta - WP_theta)
        if delta_theta <= abs(i_theta) and O_r < WP_r:
            
            # get direction modifier
            if O_theta - WP_theta < 0:
                return True, 1
            else:
                return True, -1
        else:
            return False, 0
    
    # function for calculating pathpoints around object
    def get_path_points(self, WP_r, WP_theta, O_r, O_theta, O_R, dir_mod):
        # assume WP_theta = 0
        delta_theta = abs(O_theta - WP_theta)
        A_R = self.a_mod*O_R+self.w/2

        # find PP1 and PP5
        D_R = A_R - O_r*m.sin(delta_theta)
        A_l = sqrt((A_R+D_R)^2-A_R^2)
        PP1_r = O_r*m.cos(delta_theta) - A_l
        PP5_r = O_r*m.cos(delta_theta) + A_l

        # find D1 and D2
        D1_r = m.sqrt(D_R^2+PP1_r^2)
        D1_theta = m.atan(D_R/PP1_r)
        D2_r = m.sqrt(D_R^2+PP5_r^2)
        D2_theta = m.atan(D_R/PP5_r)

        # find PP3
        PP3_r = m.sqrt(D_R^2+(O_r*m.cos(delta_theta))^2)
        PP3_theta = m.atan(D_R/(O_r*m.cos(delta_theta)))

        # find PP2 and PP4
        A_theta = m.atan(A_l/A_R)
        PP2_r = m.sqrt(O_r^2+A_R^2-2*O_r*A_R*m.cos(m.pi/2-delta_theta-A_theta))
        PP2_theta = m.asin(sin(m.pi/2-delta_theta-A_theta)*A_R/PP2_r) - delta_theta
        PP4_r = m.sqrt(O_r^2+A_R^2-2*O_r*A_R*m.cos(m.pi/2-delta_theta+A_theta))
        PP4_theta = m.asin(m.sin(m.pi/2-delta_theta+A_theta)*A_R/PP4_r) - delta_theta

        # polor cords for all points
        D1_p = [D1_r, dir_mod*D1_theta + WP_theta]
        D2_p = [D2_r, dir_mod*D2_theta + WP_theta]
        PP1_p = [PP1_r, WP_theta]
        PP2_p = [PP2_r, dir_mod*PP2_theta + WP_theta]
        PP3_p = [PP3_r, dir_mod*PP3_theta + WP_theta]
        PP4_p = [PP4_r, dir_mod*PP4_theta + WP_theta]
        PP5_p = [PP5_r, WP_theta]

        # get cords
        geod = Geodesic.WGS84.Direct(self.referance_point[1], self.referance_point[0], PP1_p[1], PP1_p[0])
        PP1 = [geod['lon2'], geod['lat2']]
        geod = Geodesic.WGS84.Direct(self.referance_point[1], self.referance_point[0], PP2_p[1], PP2_p[0])
        PP2 = [geod['lon2'], geod['lat2']]
        geod = Geodesic.WGS84.Direct(self.referance_point[1], self.referance_point[0], PP3_p[1], PP3_p[0])
        PP3 = [geod['lon2'], geod['lat2']]
        geod = Geodesic.WGS84.Direct(self.referance_point[1], self.referance_point[0], PP4_p[1], PP4_p[0])
        PP4 = [geod['lon2'], geod['lat2']]
        geod = Geodesic.WGS84.Direct(self.referance_point[1], self.referance_point[0], PP5_p[1], PP5_p[0])
        PP5 = [geod['lon2'], geod['lat2']]

        return D_R, A_R, A_, A_theta, PP1, PP2, PP3, PP4, PP5
    
    # function for getting current end point (waypoint, post, or gate)
    def set_end_point(self, pos):
        self.end_point = pos
    
    # main loop
    def run(self):
        # set reference point
        if self.obstacle_num == 0:
            self.referance_point = g.position
        else:
            for Enter in self.enter_point and Exit in self.exit_point:
                if Enter > g.pathpoint_num >= Exit:
                    self.referance_point = g.position
        
        # set endpoint if post is found
        if len(g.posts["id"]) == 0:
            self.end_point = g.waypoints[0]
        else:
            self.end_point = g.waypoints[0]
            for i in range(0, len(g.posts["id"])):
                g.debug_pub.publish("Waypoint data")
                g.debug_pub.publish("%f, %f, %d" % (g.waypoints[0][0], g.waypoints[0][1], g.waypoint_id[0]))
                g.debug_pub.publish(str(len(g.posts["id"])))
                if g.posts["id"][i] == g.waypoint_id[0]:
                    # set approximate distance if distance is not yet found
                    if len(g.posts["distance"]) != len(g.posts["id"]):
                        g.posts["distance"].append(4)
                    
                    # get cords of post
                    geod = Geodesic.WGS84.Direct(g.position[1], g.position[0], g.heading + g.posts['heading'][i], g.posts['distance'][i])
                    self.end_point = [geod['lon2'], geod['lat2']]
                    g.waypoints[0] = self.end_point
        
        
        # get end point r and theta
        geod = Geodesic.WGS84.Inverse(self.referance_point[1], self.referance_point[0], self.end_point[1], self.end_point[0])
        ep_r = geod['s12']
        ep_theta = geod['azi1']

        # reset pathpoints
        g.pathpoints["position"] = []
        g.pathpoints["linear"] = []
        g.pathpoints["heading"] = []
        g.pathpoints["radius"] = []
        g.pathpoints["speed"] = []

        # path around any objects
        for i in range(0, len(g.objects["position"])):
            # get object r and theta
            geod = Geodesic.WGS84.Inverse(self.referance_point[1], self.referance_point[0], g.objects["position"][i][1], g.objects["position"][i][0])
            o_r = geod['s12']
            o_theta = geod['azi1']

            # check if object intersects
            intersects, mod = self.check_intersection(ep_r, ep_theta, o_r, o_theta, g.objects["size"][i])

            if intersects:
                self.obstacle_num += 1

                # calculate pathpoints
                d_r, a_r, a_theta, pp1, pp2, pp3, pp4, pp5 = self.get_path_points(ep_r, ep_theta, o_r, o_theta, g.objects["size"][i], mod)

                # add path points to dictionary
                g.pathpoints["position"].append(pp1, pp2, pp3, pp4, pp5)
                g.pathpoints["linear"].append(True, False, False, False, False)
                g.pathpoints["heading"].append(ep_theta, ep_theta-mod*a_theta, ep_theta, ep_theta+mod*a_theta, ep_theta)
                g.pathpoints["radius"].append(None, d_r, a_r, a_r, d_r)
                g.pathpoints["speed"].append(80, 60, 60, 60, 60)
    
        # add endpoint to path points
        if self.end_point != None:
            g.pathpoints["position"].append(self.end_point)
            g.pathpoints["linear"].append(True)
            g.pathpoints["heading"].append(ep_theta)
            g.pathpoints["radius"].append(None)
            g.pathpoints["speed"].append(80)

        g.debug_pub.publish("Running Pathing")
