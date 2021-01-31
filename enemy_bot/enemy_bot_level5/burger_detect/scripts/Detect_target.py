import cv2
import numpy as np
import glob
import os
import argparse
eval=0.0
def BGR_to_YUV(rgb):
    m = np.array([[ 0.11400, 0.50000, -0.08131],
        [0.58700, -0.33126, -0.41869],
        [ 0.29900, -0.16874,  0.50000]])

    yuv = np.dot(rgb,m)
    yuv[:,:,1:]+=128.0
    return yuv
def hough_transforn_circle(img,file):
    global eval
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("keypoints", gray)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    circles = cv2.HoughCircles(
    gray, cv2.HOUGH_GRADIENT, dp=1.0, minDist=100, param1=80, param2=30
    )
    height=img.shape[0]
    width=img.shape[1]
    yuv=BGR_to_YUV(img)
    #print(circles)
    if circles is not None:
        circles = circles.squeeze(axis=0)
        for cx, cy, r in circles:
            xmin=int((cx-r))
            xmax=int((cx+r))
            ymin=int((cy-r))
            ymax=int((cy+r))
            if xmin<0:
                xmin=0
            if xmax >width:
                xmax=width-1
            if ymin<0:
                ymin=0
            if ymax>height:
                ymax=height-1

            h=ymax-ymin
            w=xmax-xmin
            crop=yuv[ymin:ymax,xmin:xmax]#[xmin:xmax]
            crop_Cr=crop.reshape(h*w,3)[:,2]
            Red_num=len(np.where(crop_Cr>150)[0])


            Red_rate =Red_num/float((h*w))

            if Red_rate>0.5:
                print(file)
                eval+=1
                cv2.rectangle(img,(cx-r,cy-r),(cx+r,cy+r),(0,255,0),2)
            #print(ymin,ymax,xmin,xmax)
            #print(crop.shape)
            #print(crop)

    return img
parser = argparse.ArgumentParser()
parser.add_argument("input",
        help="Input_dir."
        )
parser.add_argument("output",
        help="Output_dir."
        )
parser.add_argument("--e",
        default=".jpg",
        help="Extention of Image."
        )
args=parser.parse_args()

if not os.path.exists(args.output):
    os.mkdir(args.output)

im_dir=glob.glob(args.input+"/*"+args.e)
im_dir.sort()
for file in im_dir:
    #print(file)
    base=os.path.basename(file)
    base,_=os.path.splitext(base)
    im =cv2.imread(file)
    out =hough_transforn_circle(im,file)
    cv2.imwrite(args.output+"/"+base+"_out"+args.e,out)
print eval
#cv2.imshow("keypoints", out)
#cv2.waitKey()
