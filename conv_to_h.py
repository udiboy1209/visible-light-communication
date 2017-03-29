lut = []
lut_decode = [0 for i in range(1024)]

with open('8b_10b_coding.csv','r') as csv:
    for line in csv:
        if line[0] != ';':
            cols = line.split(',')
            lut.append([cols[0],cols[4],cols[9],cols[10],cols[15]])

print(len(lut))

with open('8b10b_lut.h','w') as header:
    header.write('static uint8_t encode_lut[1024] = {\n')
    for i,entry in enumerate(lut):
        Dp = entry[1]
        header.write('0b%s,0b%s,\n' % (Dp[0:5],Dp[5:10]))
        lut_decode[int(Dp,2)] = i
    for i,entry in enumerate(lut):
        Dn = entry[3]
        header.write('0b%s,0b%s,\n' % (Dn[0:5],Dn[5:10]))
        lut_decode[int(Dn,2)] = i
    header.write('};\n\nstatic int8_t D_lut[512] = {')
    for entry in lut:
        Dp = entry[2]
        if Dp != '0':
            Dp = '%s2' % Dp
        header.write('%s,' % Dp)
    for entry in lut:
        Dn = entry[4]
        if Dn != '0':
            Dn = '%s2' % Dn
        header.write('%s,' % Dn)
    header.write('};\n\nstatic uint8_t decode_lut[1024] = {\n')
    for entry in lut_decode:
        header.write('%d,\n' % entry)
    header.write('};\n\n')
