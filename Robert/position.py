import math, time

# constants
marker_list = [[0,718.75,5750],[1,1437.5,5750],[2,2156.25,5750],[3,2875,5750],[4,3593.75,5750],[5,4312.5,5750],[6,5031.25,5750],[7,5750,5031.25],[8,5750,4312.5],[9,5750,3593.75],[10,5750,2875],[11,5750,2156.25],[12,5750,1437.5],[13,5750,718.75],[14,5031.25,0],[15,4312.5,0],[16,3593.75,0],[17,2875,0],[18,2156.25,0],[19,1437.5,0],[20,718.75,0],[21,0,718.75],[22,0,1437.5],[23,0,2156.25],[24,0,2875],[25,0,3593.75],[26,0,4312.5],[27,0,5031.25]]
data_array = []
def get_intersections(x0, y0, r0, x1, y1, r1):
	d=math.sqrt((x1-x0)**2 + (y1-y0)**2)
	if d > r0 + r1 :
		return [[-1,-1],[-1,-1]]
	if d < abs(r0-r1):
		return [[-1,-1],[-1,-1]]
	if d == 0 and r0 == r1:
		return [[-1,-1],[-1,-1]]
	else:
		a=(r0**2-r1**2+d**2)/(2*d)
		h=math.sqrt(r0**2-a**2)
		x2=x0+a*(x1-x0)/d   
		y2=y0+a*(y1-y0)/d   
		x3=x2+h*(y1-y0)/d     
		y3=y2-h*(x1-x0)/d 
		x4=x2-h*(y1-y0)/d
		y4=y2+h*(x1-x0)/d
		return ([[x3, y3], [x4, y4]])
def get_position(R):
	'''
	inputs from robot camera in degrees and mm
	list of markers seen by camera, in order theta, psi, r, marker number
	'''
	seen_markers = []
	markers_input = R.camera.see()
	for marker in markers_input:
		seen_markers.append([math.degrees(marker.spherical.rot_x),math.degrees(marker.spherical.rot_y),marker.spherical.dist,marker.id])
	print('seen: ' + str(len(seen_markers)) + ' markers')
	if len(seen_markers) > 1:
		print('Finding position...')
		marker_ids = [x[3] for x in seen_markers]
		circles = []
		for i in seen_markers:
			radius = (i[2]*math.cos(math.radians(i[0])))
			circles.append([marker_list[i[3]][1],marker_list[i[3]][2],radius])
		valid_points = [[],[]]
		for i in range(len(circles)):
			if i + 1 != len(circles):
				a = i+1
			else:
				a = 0
			points = get_intersections(circles[i][0],circles[i][1],circles[i][2], circles[a][0],circles[a][1],circles[a][2])
			for i in points:
				if 0<i[0]<5750 and 0<i[1]<5750:
					valid_points[0].append(i[0])
					valid_points[1].append(i[1])
		if len(valid_points[0]) > 1:
			avg_x = sum(valid_points[0]) / len(valid_points[0])
			avg_y = sum(valid_points[1]) / len(valid_points[1])
			bearing_array = []
			for i in seen_markers:
				bearing = math.atan2(marker_list[i[3]][1]-avg_x,marker_list[i[3]][2]-avg_y)-math.radians(i[1])
				if bearing < math.radians(0):
					bearing+=math.radians(360)
				elif bearing >math.radians(360):
					bearing -= math.radians(360)
				bearing_array.append(bearing)
			x, y = 0, 0
			for i in bearing_array:
				x += math.cos(i)
				y += math.sin(i)
			bearing = math.degrees(math.atan2(y, x))
			if bearing < math.radians(0):
				bearing+=math.radians(360)
			elif bearing >math.radians(360):
				bearing -= math.radians(360)
			# array of necessary data in order: (coordinates array), bearing, the markers being viewed by the camera
			data_array.append([[avg_x,avg_y],bearing, marker_ids])
			return [[avg_x,avg_y],bearing, marker_ids]
		else: 
			return None
	else:
		return None
