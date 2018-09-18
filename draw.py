import matplotlib.pyplot as plt

def lineToXYZ(line):
    '''
    Input: a line from df.readline()
    Output: a list of [x,y,z] float value
    '''
    line = line.strip("\n")
    line = line.split(",")
    for i in range(3):
        line[i] = float(line[i])
    return line


file_name = './pos.txt'
df = file(file_name, 'r')
print lineToXYZ(df.readline())

def main():
    '''
    need 1+3+3+3 vectors
    t, pos, vel and acc
    '''
    time = []
    pos_x = []
    pos_y = []
    pos_z = []
    vel_x = []
    vel_y = []
    vel_z = []
    acc_x = []
    acc_y = []
    acc_z = []

    '''
    open 4 files, time, pos, vel, acc
    '''
    timef = file("./time.txt",'r')
    posf = file("./pos.txt",'r')
    velf = file("./vel.txt",'r')
    accf = file("./acc.txt",'r')

    'read the file until end'
    pos = posf.readline()
    while not pos == '':
        ti = float(timef.readline().strip('\n'))
        pos = lineToXYZ(pos)
        vel = lineToXYZ(velf.readline())
        acc = lineToXYZ(accf.readline())
        print ti, pos, vel, acc
        print '--------------'
        'append data to list'
        time.append(ti)
        pos_x.append(pos[0])
        pos_y.append(pos[1])
        pos_z.append(pos[2])

        vel_x.append(vel[0])
        vel_y.append(vel[1])
        vel_z.append(vel[2])

        acc_x.append(acc[0])
        acc_y.append(acc[1])
        acc_z.append(acc[2])
        pos = posf.readline()



    timef.close()
    posf.close()
    velf.close()
    accf.close()

    'draw curve'
    p1, = plt.plot(time, pos_x)
    p2, = plt.plot(time, pos_y)
    p3, = plt.plot(time, pos_z)
    plt.xlabel('Time: s')
    plt.ylabel('Pos: m')
    plt.legend(handles = [p1,p2,p3], labels=["pos x","pos y","pos z"], loc='best')

    plt.figure()
    v1, =plt.plot(time, vel_x)
    v2, =plt.plot(time, vel_y)
    v3, =plt.plot(time, vel_z)
    plt.xlabel('Time: s')
    plt.ylabel('Vel: m/s')
    plt.legend(handles = [v1,v2,v3], labels=["vel x","vel y","vel z"], loc='best')

    plt.figure()
    a1, =plt.plot(time, acc_x)
    a2, =plt.plot(time, acc_y)
    a3, =plt.plot(time, acc_z)
    plt.xlabel('Time: s')
    plt.ylabel('Acc: m/s^2')
    plt.legend(handles = [a1,a2,a3], labels=["acc x","acc y","acc z"], loc='best')

    plt.show()


        


    


if __name__ == '__main__':
    main()