import cv2

image_path = 'bev.png'
img = cv2.imread(image_path)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, (10, 100, 20), (25, 255, 255))

cv2.imwrite('mask.png', mask)
cv2.imshow("orange", mask)

cv2.waitKey()
cv2.destroyAllWindows()
