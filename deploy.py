import shutil, os
TARGET_DIR = 'target'
FILENAME = 'robot'
FILEEXT = 'zip'
SOURCE_DIR = 'robot'

print("Robot QOL compressor v1.0.0")
if os.path.exists(TARGET_DIR):
    print("Removing old files...")
    shutil.rmtree(TARGET_DIR)
print("Recreating target directory...")
os.makedirs(TARGET_DIR)
print("Compressing robot folder located at: " + SOURCE_DIR)
shutil.make_archive(TARGET_DIR + "/" + FILENAME, FILEEXT, SOURCE_DIR)
print("Done! You can now upload the zip file to the robot.")
print("File location: ./" + TARGET_DIR + "/robot.zip")
