file = open("temp.txt", "r")

joint_name ,joint_position = None, None
for line in file:
	if "name" in line:
		joint_name = eval(line[6:])
	if "position" in line:
		joint_position = eval(line[10:])
		d = {}
		for i in range(1, len(joint_name)-1):
			d[joint_name[i]] = joint_position[i];
		print(d)
	#print(joint_name)
	#print(joint_position)