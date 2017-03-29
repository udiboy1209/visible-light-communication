import serial
import sys,os
from time import time

conn = serial.Serial(sys.argv[1],115200,timeout=0.01)

filename = sys.argv[2]
fc = 0
fsize = os.path.getsize(filename)

t = time()

with open(filename) as f:
    while fc < fsize:
        chunk = f.read(256)

        for c in chunk:
            if not c:
                print "End of file"
                break
            conn.write(c)
            fc += 1

tt = time()-t;

print('Sent %d bytes' % fc)
print('Time taken %f s' % tt)
print('Speed %f bps' % (float(fc)*8/tt))

