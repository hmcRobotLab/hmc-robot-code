#!/usr/bin/python
#
# this script requires ROS diamondback
# for installation instructions, see http://www.ros.org 

import argparse
import sys
import os
import math

if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This scripts reads a bag file containing RGBD data, 
    adds the corresponding PointCloud2 messages, and saves it again into a bag file. 
    Optional arguments allow to select only a portion of the original bag file.  
    ''')
    parser.add_argument('--start', help='skip the first N seconds of  input bag file (default: 0.0)',default=0.00)
    parser.add_argument('--duration', help='only process N seconds of input bag file (default: off)')
    parser.add_argument('--nth', help='only process every N-th frame of input bag file (default: 15)',default=15)
    parser.add_argument('--skip', help='skip N blocks in the beginning (default: 1)', default=1)
    parser.add_argument('--compress', help='compress output bag file', action='store_true')
    parser.add_argument('inputbag', help='input bag file')
    parser.add_argument('outputbag', nargs='?',help='output bag file')
    args = parser.parse_args()

    import roslib; roslib.load_manifest('rgbd_benchmark_tools')
    import rospy
    import rosbag
    import sensor_msgs.msg
    import cv
    from cv_bridge import CvBridge, CvBridgeError
    import struct
    import tf
    
    if not args.outputbag:
        args.outputbag = os.path.splitext(args.inputbag)[0] + "-points.bag"
      
    print "Processing bag file:"
    print "  in:",args.inputbag
    print "  out:",args.outputbag
    print "  starting from: %s seconds"%(args.start)
        
    if args.duration:
        print "  duration: %s seconds"%(args.duration)
        
    print "  saving every %s-th frame"%(args.nth)
    args.skip = float(args.skip)
    print "  skipping %s blocks"%(args.skip)

    inbag = rosbag.Bag(args.inputbag,'r')
    print " starting "
    if args.compress:
        param_compression = rosbag.bag.Compression.BZ2
    else:
        param_compression = rosbag.bag.Compression.NONE
        
    depth_image = None
    rgb_image_color = None
    cortex = None

    nan = float('nan')
    bridge = CvBridge()
    frame = 0 
    written = 0
    transforms = dict()
    
    time_start = None
    tft = tf.Transformer(True)
    for topic, msg, t in inbag.read_messages():
        if time_start==None:
            time_start=t
        if t - time_start < rospy.Duration.from_sec(float(args.start)):
            continue
        if args.duration and (t - time_start > rospy.Duration.from_sec(float(args.start) + float(args.duration))):
            break
        #print "t=%f\r"%(t-time_start).to_sec(),
        if topic == "/tf":
            for transform in msg.transforms:
                tft.setTransform(transform)
            continue
        if topic == "/camera/rgb/image_color":
            rgb_image_color = msg
            continue
        if topic == "/camera/depth/image" and rgb_image_color:
            depth_image = msg
            # now process frame
            
            if depth_image.header.stamp - rgb_image_color.header.stamp > rospy.Duration.from_sec(1/30.0):
                continue
            
            frame += 1
            if frame % float(args.nth) ==0:
                if args.skip > 0:
                    args.skip -= 1
                else:
                    # store messages
                    trans = 0
                    try: 
                      trans = tft.lookupTransform('/openni_depth_frame','/odom',t)
                    except:
                      print "miss transform"
                      continue
                    print "!! !!! !!!"
                    print trans
                    x = trans[1][0]
                    y = trans[1][1]
                    z = trans[1][2]
                    w = trans[1][3]

                    filename = "out/%03i" % written
                    written += 1
                    f3d = open(filename+".3d",'w')
                    fpose = open(filename+".pose",'w')


                    fpose.write("%f %f %f\n0 0 %f" % (trans[0][0],trans[0][1],trans[0][2], math.asin(-2 (x*z - w*y))))

                    centerX = 319.5
                    centerY = 525.5
                    depthFocalLength = 525.0
                    f3d.write("%i x %i" % (depth_image.height,depth_image.width))
                    for v in range(depth_image.height):
                        for u in range(depth_image.width):
                            d = cv_depth_image[v,u]
                            rgb = cv_rgb_image_color[v,u]
                            ptx = (u - centerX) * d / depthFocalLength;
                            pty = (v - centerY) * d / depthFocalLength;
                            ptz = d;
                            if math.isnan(ptx) or math.isnan(pty) or math.isnan(ptz):
                              ptx=pty=ptz=0
                            f3d.write("%f %f %f %i %i %i" % (ptx,pty,ptz,rgb[0],rgb[1],rgb[2]))
                            buffer.append(struct.pack('ffffBBBBIII',
                                ptx,pty,ptz,1.0,
                                rgb[0],rgb[1],rgb[2],0,
                                0,0,0))
                    rgb_points.data = "".join(buffer)
                    outbag.write("/camera/rgb/points", rgb_points, t)                
                    f3d.close()
                    fpose.close()
            # consume the images
            depth_image = None
            rgb_image_color = None
            continue
    print
