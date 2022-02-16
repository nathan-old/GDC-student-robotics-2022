import os, datetime, subprocess, shutil
def get_locations():
    dir_locations = ['D:','E:'] #add possible drive locations here
    usb_location = ''
    for i in dir_locations:
        if os.path.isfile(os.path.join(i,'robot.zip')) == True:
            print('Found at location',i)
            usb_location = i
    if usb_location == '':
        print('Could not find a valid USB with robot.zip on it.')
        exit()
    

    comp_locations = ['C:\\Users\\Kimbe\\Downloads\\Backup'] #add your backup locations here
    print(comp_locations)
    comp_location = ''
    for i in comp_locations:
        if os.path.exists(i) == True:
            print('Found at location',i)
            comp_location = i

    if comp_location == '':
        print('Could not find a valid computer location')
        exit()
    day = (datetime.datetime.now().day)
    month = (datetime.datetime.now().month)
    year = str(datetime.datetime.now().year)
    hour = (datetime.datetime.now().hour)
    minute = (datetime.datetime.now().minute)
    file_name = ('Backup_'+'{0:0=2d}'.format(day)+'-'+'{0:0=2d}'.format(month)+'-'+year+'--'+'{0:0=2d}'.format(hour)+'.'+'{0:0=2d}'.format(minute))
    copy_location = os.path.join(comp_location,str(file_name))
    print(copy_location)

    return usb_location, copy_location, comp_location

src, dest, comp = get_locations()

if os.path.exists(dest) == False:
    os.mkdir(dest)
else:
    print('Already exists try later')
    exit()
subprocess.call(['xcopy', src, dest, '/C', '/h','/i','/k','/q','/r','/s'])


folders = []
for (dirpath, dirnames, filenames) in os.walk(comp):
    folders.extend(dirnames)
    print(filenames)
    break
print(folders)
#while len(folders) > 15:
#    shutil.rmtree(os.path.join(comp,folders[0]))
#    folders.remove(folders[0])


