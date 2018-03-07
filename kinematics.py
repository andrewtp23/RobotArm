import math

def convToRadians(degrees):
    pi = 3.14159265
    return (degrees * pi) / 180

def generateWorkFrame():
    r = 4
    frame = [[0 for x in range(r)] for y in range(r)]

    for x in range(r):
        for y in range(r):
            if(x == y):
                frame[x][y] = 1
            else:
                frame[x][y] = 0
    return frame

def createJointFrame(DHFrame):
    r = 4
    frame = [[0 for x in range(r)] for y in range(r)]

    frame[0][0] = math.cos(DHFrame[0])
    frame[0][1] = -1 * math.sin(DHFrame[0]) * math.cos(DHFrame[1])
    frame[0][2] = math.sin(DHFrame[0]) * math.sin(DHFrame[1])
    frame[0][3] = DHFrame[3] * math.cos(DHFrame[0])

    frame[1][0] = math.sin(DHFrame[0])
    frame[1][1] = math.cos(DHFrame[0]) * math.cos(DHFrame[1])
    frame[1][2] = -1 * math.cos(DHFrame[0]) * math.sin(DHFrame[1])
    frame[1][3] = DHFrame[3] * math.cos(DHFrame[0])

    frame[2][0] = 0
    frame[2][1] = math.sin(DHFrame[1])
    frame[2][2] = math.cos(DHFrame[1])
    frame[2][3] = DHFrame[2]

    frame[3][0] = 0
    frame[3][1] = 0
    frame[3][2] = 0
    frame[3][3] = 1

    return frame

if __name__ == '__main__':
    print(convToRadians(90))
    workframe = generateWorkFrame()

    dh1 = []
    dh1.append(0.0001)
    dh1.append(convToRadians(0.0001))
    dh1.append(169.77)
    dh1.append(64.2)

    jf1 = createJointFrame(dh1)
    print(jf1[0][0])
"""
    for x in range(4):
        for y in range(4):
            print(jf1[x][y])
        print("\n")
"""