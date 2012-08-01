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
<<<<<<< Updated upstream
=======
    from geometry_msgs.msg import TransformStamped
    from geometry_msgs.msg import Quaternion
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
        if topic == "/tf":
=======
        if topic == "odom":
            faket = TransformStamped()
            faket.header.stamp = t
            faket.header.frame_id = "/odom"
            faket.child_frame_id = "/base_link"
            #print msg.pose.pose.position.x
            faket.transform.translation.y = msg.pose.pose.position.x
            faket.transform.translation.z = msg.pose.pose.position.z
            faket.transform.rotation.x = msg.pose.pose.orientation.x
            faket.transform.rotation.y = msg.pose.pose.orientation.y
            faket.transform.rotation.z = msg.pose.pose.orientation.z
            faket.transform.rotation.w = msg.pose.pose.orientation.w
            #tft.setTransform(faket)
        if topic == "tf":
            faket = TransformStamped()
            faket.header.stamp = msg.transforms[0].header.stamp
            faket.header.frame_id = "/base_link"
            faket.child_frame_id = "/pan_base"
            faket.transform.translation.x = -.1143
            faket.transform.translation.y = 0
            faket.transform.translation.z = .4728
            faket.transform.rotation.x=0
            faket.transform.rotation.y=0
            faket.transform.rotation.z=0
            faket.transform.rotation.w=1
            tft.setTransform(faket)
            #faket = TransformStamped()
            #faket.header.stamp = msg.transforms[0].header.stamp
            #faket.header.frame_id = "/pan_base"
            #faket.child_frame_id = "/openni_camera"
            #faket.transform.translation.x = 0
            #faket.transform.translation.y = 0
            #faket.transform.translation.z = .02
            #faket.transform.rotation.x=0
            #faket.transform.rotation.y=0
            #faket.transform.rotation.z=0
            #faket.transform.rotation.w=1
            #tft.setTransform(faket)
>>>>>>> Stashed changes
            for transform in msg.transforms:
                tft.setTransform(transform)
            continue
        if topic == "/camera/rgb/image_color":
            rgb_image_color = msg
            continue
        if topic == "/camera/depth/image" and rgb_image_color:
            depth_image = msg
            # now process frame
            
            if depth_image.header.stamp - rgb_image_color.header.stamp > rospy.Duration.from_sec(5.0/30.0):
                continue
            
            frame += 1
            if frame % float(args.nth) ==0:
                if args.skip > 0:
                    args.skip -= 1
                else:
                    # store messages
                    trans = 0
                    trans2 = 0

                    #ct = [('/odom','/base_link'),('/base_link','/pan_base'),('/pan_base','/openni_camera'),('/openni_camera','/openni_depth_frame')]
                    #for x in ct:
                    #  try: 
                    #    trans = tft.lookupTransform(x[0],x[1],t)
                    #  except:
                    #    print "miss transform " + x[0] + " " + x[1] + str((t-time_start).to_sec())
                    #    continue
                    #print

                    ##trans = tft.lookupTransform('/openni_depth_frame','/odom',tft.getLatestCommonTime("/openni_depth_frame","base_link"))
                    #try: 
                    #  print "\t" + "odom base" + str(tft.lookupTransform('/base_link','/odom', rospy.Time()))
                    #except Exception as e:
                    #  print e
                    #  continue
                    #try: 
                    #  print "\t" + "odom pan" + str(tft.lookupTransform('/pan_base','/odom', rospy.Time()))
                    #except Exception as e:
                    #  print e
                    #  continue
                    #try: 
                    #  print "\t" + "odom camera" + str(tft.lookupTransform('/openni_camera','/odom', rospy.Time()))
                    #except Exception as e:
                    #  print e
                    #  continue
                    #try: 
                    #  print "\t" + "base pan" + str(tft.lookupTransform('/pan_base','/base_link', rospy.Time()))
                    #except Exception as e:
                    #  print e
                    #  continue
                    #try: 
                    #  trans = tft.lookupTransform('/openni_depth_frame','/odom', rospy.Time())
                    #  trans2 = tft.lookupTransform('/odom','/openni_depth_frame', rospy.Time())
                    #except Exception as e:
                    #  print e
                    #  print "overall error"
                    #  continue

                    try: 
                      trans = tft.lookupTransform('/odom','/openni_depth_frame', rospy.Time())
                    except Exception as e:
                      print e
                      continue
                    print
                    #print "!! !!! !!!"
                    eq= tf.transformations.euler_from_quaternion(trans[1])
                    #filename = "out%i/scan%03i" % (written / 1000, written % 1000)
                    filename = "out/scan%03i" % (written)
                    
                    written += 1
                    f3d = open(filename+".3d",'w')
                    fpose = open(filename+".pose",'w')

                    fpose.write("%f %f %f\n0 0 %f" % (trans[0][0],trans[0][1],trans[0][2], eq[2]))
                    print (t-time_start).to_sec()
                    print  "%f %f %f\n0 0 %f" % (trans[0][0],trans[0][1],trans[0][2], eq[2])
                    
                    #print  "%f %f %f" % (trans2[0][0],trans2[0][1],trans2[0][2])

                    centerX = 319.5
                    centerY = 525.5
                    depthFocalLength = 525.0
                    f3d.write("%i x %i\n" % (depth_image.height,depth_image.width))

                    cv_depth_image = bridge.imgmsg_to_cv(depth_image,"passthrough")
                    cv_rgb_image_color = bridge.imgmsg_to_cv(rgb_image_color,"bgr8")
                    for v in range(depth_image.height):
                        for u in range(depth_image.width):
                            d = cv_depth_image[v,u] 
                            rgb = cv_rgb_image_color[v,u]
                            ptx = (u - centerX) * d / depthFocalLength;
                            pty = (v - centerY) * d / depthFocalLength;
                            ptz = d;
                            if math.isnan(ptx) or math.isnan(pty) or math.isnan(ptz):
                              continue
                              ptx=pty=ptz=0
                            #f3d.write("%f %f %f %i %i %i\n" % (ptx,pty,ptz,rgb[0],rgb[1],rgb[2]))
                            f3d.write("%f %f %f\n" % (ptx,pty,ptz))
                    f3d.close()
                    fpose.close()
            # consume the images
            depth_image = None
            rgb_image_color = None
            continue
    print
