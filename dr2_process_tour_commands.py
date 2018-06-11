from gaia.cu9.ari.gaiaorbit.script import EventScriptingInterface
from gaia.cu9.ari.gaiaorbit.util import Constants
import math

#config

rootDir = 'd:/projects/astronomy/gaia_dr2/'

#code

gs = EventScriptingInterface.instance()

direction = list(gs.galacticToInternalCartesian(0, -90, 1))
up = list(gs.galacticToInternalCartesian(90, 0, 1))

class Point:
    def __init__(self,x,y):
        self.x = 1.0*x
        self.y = 1.0*y

def Spline(points):
    N   = len(points)-1
    w   =     [(points[i+1].x-points[i].x)      for i in range(0,N)]
    h   =     [(points[i+1].y-points[i].y)/w[i] for i in range(0,N)]
    ftt = [0]+[3*(h[i+1]-h[i])/(w[i+1]+w[i])    for i in range(0,N-1)]+[0]
    A   =     [(ftt[i+1]-ftt[i])/(6*w[i])       for i in range(0,N)]
    B   =     [ftt[i]/2                         for i in range(0,N)]
    C   =     [h[i]-w[i]*(ftt[i+1]+2*ftt[i])/6  for i in range(0,N)]
    D   =     [points[i].y                      for i in range(0,N)]
    return A,B,C,D

def getInternalPosition(glon,glat,distance,relative_or_absolute,height):
    cosl = math.cos(glon*math.pi/180)
    sinl = math.sin(glon*math.pi/180)
    cosb = math.cos(glat*math.pi/180)
    
    yg = distance*cosb*cosl
    xg = distance*cosb*sinl

    if relative_or_absolute == 'r':
        sinb = math.sin(glat*math.pi/180)
        zg = distance*sinb
        zg += height
    else:
        zg = height
    print('position',glon,glat,height,xg,yg,zg)
    px,py,pz = gs.galacticToEquatorial([xg * Constants.PC_TO_U, zg * Constants.PC_TO_U, yg * Constants.PC_TO_U])
    
    return [px,py,pz]

def distanceInParsecs(p1,p2):
    x = (p1[0]-p2[0])/Constants.PC_TO_U
    y = (p1[1]-p2[1])/Constants.PC_TO_U
    z = (p1[2]-p2[2])/Constants.PC_TO_U
    return math.sqrt(x*x+y*y+z*z)

def interpolate_move(current_p,p,current_speed,speed):
    
    x_inc = (p[0] - current_p[0]) / 16
    y_inc = (p[1] - current_p[1]) / 16
    z_inc = (p[2] - current_p[2]) / 16

    points = []
    for i in range(0,16+1):
        c = [
            current_p[0]+i*x_inc,
            current_p[1]+i*y_inc,
            current_p[2]+i*z_inc
        ]
        points.append(c)

    # now can interpolate between these 16 points, each with potentially a different speed.

    speed_inc = (speed - current_speed)/16

    for s in range(1,16+1):
        sv = current_speed + speed_inc*s
        parsecs = distanceInParsecs(points[s-1],points[s])

        number_of_frames = int(parsecs/sv)
        # output = ','.join(list(map(str,[parsecs,speed_inc,s,sv,number_of_frames])))
        # gp.write('#'+output+"\n")
        x_inc = (points[s][0] - points[s-1][0]) / number_of_frames
        y_inc = (points[s][1] - points[s-1][1]) / number_of_frames
        z_inc = (points[s][2] - points[s-1][2]) / number_of_frames
        for frame in range(1,number_of_frames+1):
            interpolated_position = []
            x = points[s-1][0]+frame*x_inc
            y = points[s-1][1]+frame*y_inc
            z = points[s-1][2]+frame*z_inc
            output = ' '.join(list(map(str,[0,x,y,z]+direction+up)))
            dat.write(output+"\n")

def compute_cubic(x,j,cubic):
    #print('cubic',cubic)
    a,b,c,d = cubic
    return a*(x-j)**3 + b*(x-j)**2 + c*(x-j) + d

def interpolate(j,c):

    global current_speed
    global current_p

    # now can interpolate between these 16 points, each with potentially a different speed.

    current_speed = speeds[j]

    speed_inc = (speeds[j+1] - speeds[j])/16

    print('j',j)

    for s in range(1,16+1):
        print('s',s)
        sv = current_speed + speed_inc*s
        # print('speed',sv)
        s1 = 1.0*j + (s-1)/16.0
        x1 = compute_cubic(s1,j,[c[0][0][j],c[0][1][j],c[0][2][j],c[0][3][j]])
        y1 = compute_cubic(s1,j,[c[1][0][j],c[1][1][j],c[1][2][j],c[1][3][j]])
        z1 = compute_cubic(s1,j,[c[2][0][j],c[2][1][j],c[2][2][j],c[2][3][j]])

        # print('in interpolate, s1',s1,x1,y1,z1)
        s2 = 1.0*j + s/16.0
        x2 = compute_cubic(s2,j,[c[0][0][j],c[0][1][j],c[0][2][j],c[0][3][j]])
        y2 = compute_cubic(s2,j,[c[1][0][j],c[1][1][j],c[1][2][j],c[1][3][j]])
        z2 = compute_cubic(s2,j,[c[2][0][j],c[2][1][j],c[2][2][j],c[2][3][j]])

        # print('in interpolate, s2',s2,x2,y2,z2)

        parsecs = distanceInParsecs([x1,y1,z1],[x2,y2,z2])
        # print('parsecs',parsecs)

        number_of_frames = int(parsecs/sv)
        frame_inc = 1.0 / (16.0*number_of_frames)

        # print('frame data',number_of_frames,frame_inc)

        for frame in range(0,number_of_frames):
            s3 = s1 + frame * frame_inc
            x = compute_cubic(s3,j,[c[0][0][j],c[0][1][j],c[0][2][j],c[0][3][j]])
            y = compute_cubic(s3,j,[c[1][0][j],c[1][1][j],c[1][2][j],c[1][3][j]])
            z = compute_cubic(s3,j,[c[2][0][j],c[2][1][j],c[2][2][j],c[2][3][j]])
            a1 = compute_cubic(s3,j,[c[3][0][j],c[3][1][j],c[3][2][j],c[3][3][j]])
            a2 = compute_cubic(s3,j,[c[4][0][j],c[4][1][j],c[4][2][j],c[4][3][j]])
            if a2 != 0:
                print('angles',a1,a2)

            direction_new = gs.rotate3(direction, up_axis, a1)
            axis = gs.cross3(up, direction_new)
            direction_new = list(gs.rotate3(direction_new, axis, a2))
            up_new = gs.rotate3(up, up_axis, a1)
            axis = gs.cross3(up_new, direction_new)
            up_new = list(gs.rotate3(up_new, axis, a2))
            output = ' '.join(list(map(str,[0,x,y,z]+direction_new+up_new)))
            current_p = [x,y,z]
            dat.write(output+"\n")

def interpolate_rotate(current_p,p,angle1,angle2,frames):
    global direction
    global up

    angle1_inc = angle1/frames
    angle2_inc = angle2/frames
    x_inc = (p[0] - current_p[0]) / frames
    y_inc = (p[1] - current_p[1]) / frames
    z_inc = (p[2] - current_p[2]) / frames
    for frame in range(1,frames+1):
        x = current_p[0]+frame*x_inc
        y = current_p[1]+frame*y_inc
        z = current_p[2]+frame*z_inc

        direction = gs.rotate3(direction, up_axis, angle1_inc)
        axis = gs.cross3(up, direction)
        direction = list(gs.rotate3(direction, axis, angle2_inc))
        up = gs.rotate3(up, up_axis, angle1_inc)
        axis = gs.cross3(up, direction)
        up = list(gs.rotate3(up, axis, angle2_inc))
        #direction = list(gs.galacticToInternalCartesian(0+frame*angle1_inc, -90+frame*angle2_inc, 1))
        #up = list(gs.galacticToInternalCartesian(90+frame*angle1_inc, 0+frame*angle2_inc, 1))
        output = ' '.join(list(map(str,[0,x,y,z]+direction+up)))
        #print(output)
        dat.write(output+"\n")

height_above_galactic_plane = 800
current_height = height_above_galactic_plane
current_speed = 1 #pc per frame
current_angle1 = 0
current_angle2 = 0
source_list = []
interpolated = []
speeds = []

command_length = {'m':8,'p':10,'d':2,'j':2,'r':5,'c':3,'t':3}
commands = []

# up_axis = list(gs.galacticToInternalCartesian(0, 90, 1))
# centre_axis = list(gs.galacticToInternalCartesian(90, 0, 1))

up_axis = list(gs.galacticToInternalCartesian(0, 90, 1))
centre_axis = list(gs.galacticToInternalCartesian(90, 0, 1))

# TO DO: add more sanity checks when parsing script
error = False
fn = rootDir+'output/gaiasky/tour.bsv'
fp = open(fn,'r')
line = fp.readline()
while len(line) != 0:
    bits = line.strip().split('|')
    if len(bits) > 1:
        command = bits[0]
        if command in command_length:
            if len(bits) == command_length[command]:
                commands.append(bits)
            else:
                error = True
                break
        else:
            error = True
    line = fp.readline()
fp.close()

control_points = []
end_points = []

interpolating = False
if not error:

    #interpret script

    #fn = 'C:/Users/Jardine/.gaiasky/camera/tour_postprocess.bsv'
    #gp = open(fn,'w')

    fn = 'C:/Users/Jardine/.gaiasky/camera/tour_test.dat'
    dat = open(fn,'w')

    gs.setCameraFree()

    for command in commands:
        action = command[0]
        if action == 'p':
            name,glon,glat,r,relative_or_absolute,height,angle1,angle2,speed = command[1:]
            if height == '-':
                height = current_height
            else:
                height = float(height)
            if speed == '-':
                speed = current_speed
            else:
                speed = float(speed)

            if angle1 == '-':
                angle1 = current_angle1
            else:
                angle1 = float(angle1)

            if angle2 == '-':
                angle2 = current_angle2
            else:
                angle2 = float(angle2)
            
            glon, glat, r = float(glon),float(glat),float(r)

            p = getInternalPosition(glon,glat,r,relative_or_absolute,height)
            control_points.append(['p',p[0],p[1],p[2],angle1,angle2])
            speeds.append(speed)
            # output = ','.join(list(map(str,[name,glon,glat,r,height,speed,p[0],p[1],p[2]])))
            # gp.write('#'+output+"\n")
            
            current_p = p
            current_height = height
            current_speed = speed
            current_angle1 = angle1
            current_angle2 = angle2
        if action == 'm':
            name,glon,glat,r,relative_or_absolute,height,speed = command[1:]
            if height == '-':
                height = current_height
            else:
                height = float(height)
            if speed == '-':
                speed = current_speed
            else:
                speed = float(speed)
            
            glon, glat, r = float(glon),float(glat),float(r)

            p = getInternalPosition(glon,glat,r,relative_or_absolute,height)

            end_points.append(['m',current_p,p,current_speed,speed])

            current_p = p
            current_height = height
            current_speed = speed
        elif action == 'r':
            angle1,angle2,height,frames = command[1:]
            if height == '-':
                height = current_height
            else:
                height = float(height)
            p = getInternalPosition(glon,glat,r,'a',height)
            interpolate_rotate(current_p,p,float(angle1),float(angle2),int(frames))
    #gp.close()
else:
    print('oops, tour command parse error')

#cubic interpolation

points = {}

cubics = {}

for i in range(0,5):
    points[i] = []
    for j in range(0,len(control_points)):
        points[i].append(Point(j,control_points[j][i+1]))

for i in range(0,5):
    cubics[i] = Spline(points[i])

x,y,z = control_points[0][1:4]
output = ' '.join(list(map(str,[0,x,y,z]+direction+up)))
for j in range(0,len(control_points)-1):
    interpolate(j,cubics)

for end_point in end_points:
    action,current_p,p,current_speed,speed = end_point
    interpolate_move(current_p,p,current_speed,speed)
