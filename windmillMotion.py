
#  select a valid video file  with proper visibility;
#  with proper valid video it will show the swinging motion of the turbine;
#  press 'q' to quit
#  in order to find the angle, press 'p' , frame will be paused,
#  then make three points with your mouse click ;
#  first click in the center then on the two wings, angle will be visible on the image




import os
import math

try:
    import filetype
except:
    os.system('pip install filetype')
    import filetype
try:
    import cv2
except:
    os.system('pip install opencv-python')
    import cv2
try:
    import easygui
except:
    os.system('pip install easygui')
    import easygui
try:
    import imutils
except:
    os.system('pip install imutils')
    import imutils


#  function to draw a right arrow after determining
#  the swinging motion
def drawArrowRight(frame,mid):
    print('[+]Anti Clockwise Movement')
    end_point=(mid[0]+100,mid[1])
    cv2.arrowedLine(frame, mid, end_point,(0, 0, 255) , 9, tipLength=0.5)


#  function to draw a right arrow after determining
#  the swinging motion
def drawArrowLeft(frame,mid):
    print('[+]Clockwise Movement')
    end_point = (mid[0] - 100, mid[1])
    cv2.arrowedLine(frame, mid, end_point, (0, 0, 255), 9, tipLength=0.5)


#  detecting swinging motion direction
def check(X_coordinate):
    counter = 0
    while counter < len(X_coordinate) - 1:
      if X_coordinate[counter] > X_coordinate[counter+1]:
        return True
      return False
      count=+1


#  gradiant to find anlge
def findGradient(pt1,pt2):
  return (pt2[1]-pt1[1])/(pt2[0]-pt1[0])


#  calculating anlge
def findAngle(pointLIST):
  pt1 = pointLIST[0]
  pt2 = pointLIST[1]
  pt3 = pointLIST[2]
  print(pt1,pt2,pt3)
  m1=findGradient(pt1,pt2)
  m2=findGradient(pt1,pt3)
  angleR=math.atan((m2-m1)/(1+(m2*m1)))
  angleD=round(math.degrees(angleR))
  angleD=abs(abs(angleD)-180)
  putText(angleD,pt1,pt2)


#  Showing anlge on the frame and saving image
def putText(angle,pt1,pt2):
  cv2.putText(resizedFrame, str(angle), (pt1[0] - 100, pt1[1] - 20), cv2.FONT_HERSHEY_COMPLEX, 1.5, (0, 0, 255), 2)
  cv2.imshow('frams', resizedFrame)
  cv2.imwrite('angle&direction.png',resizedFrame)


#  function to pause video
def pauseVideo(key,frame):
  if key == ord('p'):
    cv2.setMouseCallback('All contours with bounding box',mousePoints)
    cv2.waitKey(-1)
    exit()
    #cv2.imshow('paused',frame)


#  function to draw lines for angle
def drawLines(x,y,size):
  cv2.line(resizedFrame, tuple(points_List[round((size - 1) / 3) * 3]), (x, y), (0, 0, 255), 3)


#mouse click event
def mousePoints(event, X,Y,flags,parameters):
  if event==cv2.EVENT_LBUTTONDOWN:
    size=len(points_List)
    cv2.circle(resizedFrame,(X,Y),5,(0,0,255),cv2.FILLED)
    points_List.append([X,Y])
    drawLines(X,Y,size)

    if len(points_List)==3:
      #print(points_List)
      findAngle(points_List)


def openFile():

  #  openning file
  path = easygui.fileopenbox(msg='Select a video', default='turbine2.mp4')
  if path == None:
    print('[-] No File selected  ')
    exit(1)
  if filetype.is_video(path):
    print('[+]Valid File')
    return path
  else:
    print('[-]Invalid file')
    exit(1)

#--------------------------------------------------------------------------------------------------------------------#



X_coordinate=[]
points_List=[]

#  template wing image to detect wing movement
templateImage = cv2.imread('wing.png')

#  Create a VideoCapture object and read from input file
path=openFile()

cap = cv2.VideoCapture(path)
count=0

# Check if camera opened successfully
if (cap.isOpened()== False):
  print("Error opening video stream or file")

# Read until video is completed
while(cap.isOpened()):
  #  Capture frame-by-frame
  ret, frame = cap.read()
  if ret == True:
    count=count+1
    # Display the resulting frame
    resizedFrame = imutils.resize(frame, width=500)

    src = cv2.GaussianBlur(resizedFrame, (3, 3), 0)
    gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    dst = cv2.Canny(gray, 150, 300, None, 3)

    #  Initiate ORB detector
    orb = cv2.ORB_create()

    # find the keypoints and descriptors with ORB
    kp1, des1 = orb.detectAndCompute(templateImage, None)
    kp2, des2 = orb.detectAndCompute(resizedFrame, None)

    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    # Match descriptors.
    matches = bf.match(des1, des2)
    # Sort them in the order of their distance.
    matches = sorted(matches, key=lambda x: x.distance)
    for kp in kp2:
      for match in matches:
        x,y=kp2[match.trainIdx].pt
        x=int(x)
        y=int(y)
    X_coordinate.append(x)

    (h, w) = resizedFrame.shape[:2]  # w:image-width and h:image-height
    midpoint=(w // 2, h // 2)

    # Draw first 10 matches.
    img3 = cv2.drawMatches(templateImage, kp1, resizedFrame, kp2, matches[:1], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    if count>10:
      if check(X_coordinate):
        drawArrowRight(resizedFrame, midpoint)
      else:
        drawArrowLeft(resizedFrame,midpoint)
    cv2.imshow('All contours with bounding box', resizedFrame)
    count=count+1

    #   Press Q on keyboard to  exit
    key = cv2.waitKey(1)
    pauseVideo(key,resizedFrame)

    if key == ord('q'):
        break

    #  Break the loop
  else:
    break

 # print(X_coordinate)
#   When everything done, release the video capture object
cap.release()

# Closes all the frames
cv2.destroyAllWindows()
