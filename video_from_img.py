import cv2
import glob

base_path = input("Path to usb/images folder >")
img_array = []
size = None
for filename in glob.glob(base_path + '/*.png'):
    try:
        print("Importing {}...".format(filename))
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width,height)
        img_array.append(img)
    except Exception as error:
        print("(Can ignore) Failed to read img: {}, got error: {}".format(filename, error))

#out = cv2.VideoWriter('project.avi',cv2.VideoWriter_fourcc(*'DIVX'), 2, size)
out = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'MP4V'), 2, size)

for i in range(len(img_array)):
    print(str(round((i/len(img_array)*100),2))+'%')
    out.write(img_array[i])
out.release()