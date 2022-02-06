import cv2
import glob
 
img_array = []
for filename in glob.glob('*.jpg'):
    img = cv2.imread(filename)
    height, width, layers = img.shape
    size = (width,height)
    img_array.append(img)

#out = cv2.VideoWriter('project.avi',cv2.VideoWriter_fourcc(*'DIVX'), 2, size)
out = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'MP4V'), 10, size)

for i in range(len(img_array)):
    print(str(round((i/len(img_array)*100),2))+'%')
    out.write(img_array[i])
out.release()