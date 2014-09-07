
#!/usr/bin/env python2
import roslib
import rospy
import cv2
import cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#								export ROS_MASTER_URI = "http://leonardo:11311"

class ImageConverter:

	def __init__(self):
		print "init1"
		self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image, self.callback, queue_size = 1)
		print "init2"
		self.bridge = CvBridge()
		capture = imgage_sub
		#Creates the visuals for us, don't need this for the robots internal workings
		frame = cv.QueryFrame(capture)
		frame_size = cv.GetSize(frame)
		test=cv.CreateImage(cv.GetSize(frame),8,3)
		img2=cv.CreateImage(cv.GetSize(frame),8,3)
		cv.NamedWindow("Real",0)
		cv.NamedWindow("Threshold",0)

	def callback(self, data):
		print "callback"
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print e
		print "entering while"
		while(1):
			i=0
			while(i<10):
				yellowY = colourWorkings('y')
				i = i+1
			i=0
			while(i<10):
				blueY = colourWorkings('b')
				i = i+1
			i=0
			while(i<10):
				greenY =colourWorkings('g')
				i = i+1
			i=0
			while(i<10):
				pinkY = colourWorkings('p')
				i = i+1

		#	ALL OF THIS IS FOR TESTING, SO THIS UGLY CODE IS GOIN AWAY!
			if checkYcoords(yellowY,pinkY):
				cv.Line(result,(300,50),(300,250),(0,255,255),4,8,0)
				cv.Line(result,(300,250),(300,450),(180,105,255),4,8,0)
			if checkYcoords(pinkY,yellowY):
				cv.Line(result,(350,50),(350,250),(180,105,255),4,8,0)
				cv.Line(result,(350,250),(350,450),(0,255,255),4,8,0)
			if checkYcoords(pinkY,blueY):
				cv.Line(result,(400,50),(400,250),(180,105,255),4,8,0)
				cv.Line(result,(400,250),(400,450),(255,0,0),4,8,0)
			if checkYcoords(blueY,pinkY):
				cv.Line(result,(450,50),(450,250),(255,0,0),4,8,0)
				cv.Line(result,(450,250),(450,450),(180,105,255),4,8,0)
			if checkYcoords(yellowY,greenY):
				cv.Line(result,(250,50),(250,250),(0,255,255),4,8,0)
				cv.Line(result,(250,250),(250,450),(0,255,0),4,8,0)
			if checkYcoords(greenY,yellowY):
				cv.Line(result,(200,50),(200,250),(0,255,0),4,8,0)
				cv.Line(result,(200,250),(200,450),(0,255,255),4,8,0)
			if checkYcoords(pinkY,greenY):
				cv.Line(result,(150,50),(150,250),(180,105,255),4,8,0)
				cv.Line(result,(150,250),(150,450),(0,255,0),4,8,0)
			if checkYcoords(greenY,pinkY):
				cv.Line(result,(100,50),(100,250),(0,255,0),4,8,0)
				cv.Line(result,(100,250),(100,450),(180,105,255),4,8,0)
			cv.ShowImage("RESULT",result)
			if Cpressed():
				break
			result=cv.CreateImage(cv.GetSize(frame),8,3)
			cv.NamedWindow("RESULT",0)

	def getthresholdedimg(im,whichColor):
		# Convert from RGB to HSV
		imghsv=cv.CreateImage(cv.GetSize(im),8,3)
		cv.CvtColor(im,imghsv,cv.CV_BGR2HSV)
		# Image to be returned
		imgthreshold=cv.CreateImage(cv.GetSize(im),8,1)
		# Make the thresholded image
		img=cv.CreateImage(cv.GetSize(im),8,1)
		if whichColor == 'y':
			cv.InRangeS(imghsv,cv.Scalar(0,200,200),cv.Scalar(225,255,255),img)	# Range of Yellow colours
			cv.Add(imgthreshold,img,imgthreshold)
		if whichColor == 'b':
			cv.InRangeS(imghsv,cv.Scalar(30,80,120),cv.Scalar(120,205,255),img)	# Range of Blue colours
			cv.Add(imgthreshold,img,imgthreshold)
		if whichColor == 'g':
			cv.InRangeS(imghsv,cv.Scalar(50,85,55),cv.Scalar(120,240,180),img)	# Range of Green colours
			cv.Add(imgthreshold,img,imgthreshold)
		if whichColor == 'p':
			cv.InRangeS(imghsv,cv.Scalar(140,100,130),cv.Scalar(255,180,240),img)	# Range of Pink colours
			cv.Add(imgthreshold,img,imgthreshold)
		return imgthreshold

	def colourWorkings(color):
	#	THE SAME BASIS FOR ALL COLOURS
		color_image = cv.QueryFrame(capture)
		imdraw=cv.CreateImage(cv.GetSize(frame),8,3)
		cv.SetZero(imdraw)
		cv.Flip(color_image,color_image,1)
		cv.Smooth(color_image, color_image, cv.CV_GAUSSIAN, 3, 0)
	#	BASED ON THE COLOR SPESIFIED
		imgCOLORthresh=getthresholdedimg(color_image,color)
		cv.Erode(imgCOLORthresh,imgCOLORthresh,None,3)
		cv.Dilate(imgCOLORthresh,imgCOLORthresh,None,10)
		img2=cv.CloneImage(imgCOLORthresh)
		storage = cv.CreateMemStorage(0)
		contour = cv.FindContours(imgCOLORthresh, storage, cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)	
		while contour:
	#		Find bounding rectangle
			bound_rect = cv.BoundingRect(list(contour))
			pt1 = (bound_rect[0], bound_rect[1])
			pt2 = (bound_rect[0] + bound_rect[2], bound_rect[1] + bound_rect[3])
	#		Find the boundRect center
	 		posx=cv.Round((pt1[0]+pt2[0])/2)
	 		posy=cv.Round((pt1[1]+pt2[1])/2)
	#		the y coord's what's compared
			centerspot = [posx,posy]
	#		Draw circle in the boundRect center for visual aid
	 		cv.Circle(color_image,(posx,posy),7,(200,200,200),2)
	 		cv.Circle(color_image,(posx,posy),10,(80,80,80),2)
	#		Visuals for us, for robot it is return statement
			cv.ShowImage("Real",color_image)
			cv.ShowImage("Threshold",img2)
			return posy

	def Cpressed():
		if cv.WaitKey(33)==1048603:
			cv.DestroyWindow("Real")
			cv.DestroyWindow("Threshold")
			return True

	def checkYcoords(spottop,spotbottom):
		if spottop > spotbottom:
			return True
		return False
	

def main(args):
	ic = ImageConverter()
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

	cv2.destroyAllWindows()

if __name__ == '__main__': main(sys.argv)
