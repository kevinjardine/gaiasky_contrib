This script is run inside Gaia Sky and accepts a list of bar-separated control points and uses cubic spline interpolation to generate a smooth Gaia Sky camera path between them.

The format for each control point is:

p|name|glon|glat|distance|relative_or_absolute|height|yaw|pitch|speed

where:

p is the command name 'p'

name is the name of the object (just for documentation, not used by the system)

glon and glat are the galactic coordinates for the object

distance is the distance in parsecs

relative_or_absolute is either 'r' or 'a'

height in parsecs is used to set the camera position. If relative_or_absolute is 'r' the camera is set height parsecs above the object.
If relative_or_absolute is 'a', the camera is set height parsecs above the galactic plane, and in the same xg and yg position as the object

yaw is the left/right camera angle

pitch is the up/down camera angle

speed is the current speed of the camera in parsecs per frame

the height, yaw and pitch fields can have the value '-' in which case the script will use the previous or default values

the default yaw and pitch values are 0 and 0 - the camera orientation is pointing straight down towards the galactic plane with the galactic centre to the right.

Example:

```
p|Earth|0|90|0|a|800|-|-|2
p|Wezen|238.4175|-08.2666|492.6108|a|0|-|0|2
p|3064486516048361216|227.890051917|15.314256569|869.565|a|800|-|-|3
p|3046553348261368576|224.293457960|-1.139503773|1149.4|a|800|-|-45|3
p|2921466858173578112|236.233021550|-9.056442582|1515|a|800|-|-60|2
p|5532906406922172160|258.761626748|-6.749197466|2041|a|800|-|-45|3
p|5350363910256783744|287.410278338|-0.574203729|2617.13|a|800|-|0|3
p|5256686653442846336|283.526137598|-3.966709504|1136|a|800|-|-|3
p|6059710332229923456|301.802879077|3.526195234|813|a|800|-|-|3
p|5937951995124879232|336.766173949|-1.652332380|1087|a|800|-|-|3
p|4066022591147527552|6.008906894|-1.204998593|1177|a|800|-|-|3
p|4146599166888812032|16.942776136|0.842243970|1887|a|800|-|-|3
```

You will need to edit the script before running it to point to your control point input file and camera path output file.

