import json
import cv2
import glob

base_path = input("Path to usb/debug folder >")

debug_data = []

for filename in glob.glob(base_path + '/*.json'):
    try:
        print("Importing {}...".format(filename))
        with open(filename, 'r') as debug_file:
            raw = debug_file.readline()
            data = json.loads(raw)
            debug_data.append(data)

    except Exception as error:
        print("(Can ignore) Failed to read debug file: {}, got error: {}".format(filename, error))

print("Imported {} debug log entrys".format(len(debug_data)))
output = json.dumps(debug_data, indent=4)
with open("debug_data.txt", "w") as file:
    file.write(output)
