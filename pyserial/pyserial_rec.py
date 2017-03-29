import serial
import sys
from time import time

conn = serial.Serial(sys.argv[1],115200,timeout=0.01)

filename = sys.argv[2]
fsize = 0
fc = 0
size = 0

t = time()

with open(filename,'w') as f:
    while size<4:
        r = conn.read(1)
        if not r:
            continue
        fsize = fsize*256 + ord(r)
        size += 1

    print('Want %d bytes' % fsize)

    while fc < fsize:
        r = conn.read(1)
        if not r:
            continue
        f.write(r)
        fc += 1
        print 'Received %d bytes\r' % fc,

    f.close()

tt = time()-t;

print('Received %d bytes' % fc)
print('Time taken %f s' % tt)
print('Speed %f bps' % (float(fc)*8/tt))

