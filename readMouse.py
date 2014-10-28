import struct
file = open ("/dev/input/mice","rb");

def getMouseEvent():
	buf = file.read(3);
	x,y = struct.unpack("bb", buf[1:] );
	print ("x:%d, y:%d\n" % (x,y));

while(1):
	getMouseEvent()

file.close;
