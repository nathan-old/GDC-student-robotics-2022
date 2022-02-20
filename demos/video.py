import cv2
import glob, os
import sys
from PIL import Image,ImageFont,ImageDraw

if input('Video? ').upper() == 'Y':
    if os.path.exists(os.path.join('D:','figs')) == False:
        os.mkdir(os.path.join('D:','figs'))



    plt_num = 0

    ##num = len(glob.glob(os.path.join('D:','plts','*')))

    files = []
    for (dirpath, dirnames, filenames) in os.walk(os.path.join('D:','captures')):
        files.extend(filenames)
        break
    figures = []
    for (dirpath, dirnames, filenames) in os.walk(os.path.join('D:','plts')):
        figures.extend(filenames)
        break
    files.sort()
    figures.sort()
    for i in range(len(files)):
        print(str(round((i/len(files)*100),2))+'%')
        
        images = [Image.open(os.path.join('D:','plts',figures[plt_num])),Image.open(os.path.join('D:','captures',files[i]))]
        widths, heights = zip(*(i.size for i in images))
        
            
        total_width = sum(widths)
        max_height = max(heights)

        picture = Image.new('RGB', (total_width, max_height))
        font = ImageFont.truetype(font="C:\Windows\Fonts\consola.ttf", size = 30)
        draw = ImageDraw.Draw(picture)
        x_offset = 0
        for im in images:
          picture.paste(im, (x_offset,0))
          x_offset += im.size[0]
        draw.text((5, 0),('filename: '+figures[plt_num]),(0,0,0), font=font)
        draw.text((5, 35),('filename: '+files[i]),(0,0,0), font=font)
##        print(files[i],figures[plt_num])
        picture.save(os.path.join('D:','figs','fig'+str(i)+'.png'))
        if 'A' in files[i]:
            plt_num += 1

    img_array = []
    for filename in glob.glob(os.path.join('D:','figs','*')):
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width,height)
        img_array.append(img)
        img_array.append(img)


    out = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'MP4V'), 10, size)

    for i in range(len(img_array)):
        print(str(round((i/len(img_array)*100),2))+'%')
        out.write(img_array[i])
    out.release()

    files = []
    for (dirpath, dirnames, filenames) in os.walk(os.path.join('D:','figs')):
        files.extend(filenames)
        break
    while len(files) > 0:
        os.remove(os.path.join('D:','figs',files[0]))
        files.remove(files[0])
    os.rmdir(os.path.join('D:','figs'))

if input('delete? ').upper() == 'Y':
    if os.path.exists(os.path.join('D:','figs')) == True:
        files = []
        for (dirpath, dirnames, filenames) in os.walk(os.path.join('D:','figs')):
            files.extend(filenames)
            break
        while len(files) > 0:
            os.remove(os.path.join('D:','figs',files[0]))
            files.remove(files[0])
        os.rmdir(os.path.join('D:','figs'))
    files = []
    for (dirpath, dirnames, filenames) in os.walk(os.path.join('D:','plts')):
        files.extend(filenames)
        break
##    print(files)
    files.remove(files[files.index('figure300.png')])
    while len(files) > 0:
        os.remove(os.path.join('D:','plts',files[0]))
        files.remove(files[0])

    files = []
    for (dirpath, dirnames, filenames) in os.walk(os.path.join('D:','captures')):
        files.extend(filenames)
        break
    while len(files) > 0:
        os.remove(os.path.join('D:','captures',files[0]))
        files.remove(files[0])

if input('open? ').upper() == 'Y':
    class Video(object):
        def __init__(self,path):
            self.path = path

        def play(self):
            from os import startfile
            startfile(self.path)

    class Movie_MP4(Video):
        type = "MP4"
    class log_file(Video):
        type = "txt"

    Movie_MP4(os.path.join('D:','output.mp4')).play()
##    log_file(os.path.join('D:','log.txt')).play()

