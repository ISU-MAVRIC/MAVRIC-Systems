
from time import sleep
from imutils.video import VideoStream
import cv2

from onvif import ONVIFCamera
import zeep

dir_save = "/home/mavric/MAVRIC-Systems/src/mavric/src/PTZ_Control/PTZ_Photos/"

vs = VideoStream('rtsp://admin:mavric-camera@192.168.1.64:554/out.h264').start()
frame = vs.read()

XMAX = 1
XMIN = -1
YMAX = 1
YMIN = -1



def zeep_pythonvalue(self, xmlvalue):
    return xmlvalue


def perform_move(ptz, request, timeout):
    # Start continuous move
    ptz.ContinuousMove(request)
    # Wait a certain time
    sleep(timeout)
    # Stop continuous move
    ptz.Stop({'ProfileToken': request.ProfileToken})


def move_up(ptz, request, timeout):
    print('move up...')
    request.Velocity.PanTilt.x = 0
    request.Velocity.PanTilt.y = YMAX
    perform_move(ptz, request, timeout)


def move_down(ptz, request, timeout):
    print('move down...')
    request.Velocity.PanTilt.x = 0
    request.Velocity.PanTilt.y = YMIN
    perform_move(ptz, request, timeout)


def move_right(ptz, request, timeout):
    print('move right...')
    request.Velocity.PanTilt.x = XMAX
    request.Velocity.PanTilt.y = 0

    perform_move(ptz, request, timeout)


def move_left(ptz, request, timeout):
    print('move left...')
    request.Velocity.PanTilt.x = XMIN
    request.Velocity.PanTilt.y = 0
    perform_move(ptz, request, timeout)


def zoom_up(ptz,request, timeout):
    print('zoom up')
    request.Velocity.Zoom.x = 1
    request.Velocity.PanTilt.x = 0
    request.Velocity.PanTilt.y = 0
    perform_move(ptz,request, timeout)


def zoom_dowm(ptz,request, timeout):
    print('zoom down')
    request.Velocity.Zoom.x = -1
    request.Velocity.PanTilt.x = 0
    request.Velocity.PanTilt.y = 0
    perform_move(ptz, request, timeout)


def Pano():
    mycam = ONVIFCamera('192.168.1.64', 80, 'admin', 'mavric-camera')
    # Create media service object
    media = mycam.create_media_service()
    # Create ptz service object
    ptz = mycam.create_ptz_service()

    # Get target profile
    zeep.xsd.simple.AnySimpleType.pythonvalue = zeep_pythonvalue
    media_profile = media.GetProfiles()[0]

    # Get PTZ configuration options for getting continuous move range
    request = ptz.create_type('GetConfigurationOptions')
    request.ConfigurationToken = media_profile.PTZConfiguration.token
    ptz_configuration_options = ptz.GetConfigurationOptions(request)

    request = ptz.create_type('ContinuousMove')
    request.ProfileToken = media_profile.token
    ptz.Stop({'ProfileToken': media_profile.token})

    if request.Velocity is None:
        request.Velocity = ptz.GetStatus({'ProfileToken': media_profile.token}).Position
        request.Velocity = ptz.GetStatus({'ProfileToken': media_profile.token}).Position
        request.Velocity.PanTilt.space = ptz_configuration_options.Spaces.ContinuousPanTiltVelocitySpace[0].URI
        request.Velocity.Zoom.space = ptz_configuration_options.Spaces.ContinuousZoomVelocitySpace[0].URI

    # Get range of pan and tilt
    # NOTE: X and Y are velocity vector
    global XMAX, XMIN, YMAX, YMIN
    XMAX = ptz_configuration_options.Spaces.ContinuousPanTiltVelocitySpace[0].XRange.Max
    XMIN = ptz_configuration_options.Spaces.ContinuousPanTiltVelocitySpace[0].XRange.Min
    YMAX = ptz_configuration_options.Spaces.ContinuousPanTiltVelocitySpace[0].YRange.Max
    YMIN = ptz_configuration_options.Spaces.ContinuousPanTiltVelocitySpace[0].YRange.Min

    # Rotate camera and take a picture 5 times

    for i in range(2):

        sleep(3)

        frame = vs.read()
        filename = dir_save + "Picture" + str(i+1) + ".jpg"
        cv2.imwrite(filename, frame)

        move_right(ptz, request, 0.8)


    # Rotate camera back to original position

    move_left(ptz, request, 3)

if __name__ == '__main__':
    try:
        Pano()
    except KeyboardInterrupt:

        vs.stop()
        cv2.destroyAllWindows()
