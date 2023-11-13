import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
import datetime

from Adafruit_IO import Client, Feed

ADAFRUIT_IO_USERNAME = "adaboss"
ADAFRUIT_IO_KEY = "aio_ARyQ34JqUAXGSjYzZIxD3LFxFq0e"

# Create an instance of the REST client.
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)




# Init All Value End #

node =int(input('Node vector step : '))
cap = cv2.VideoCapture(input('Vidoe source : '))
PER = int(input('Size Resolution (%) : '))

ret, img = cap.read()
prev = img

st = (0, 0)
en = (img.shape[1], img.shape[0])
stL = st
enL = en

MeanVelocity = 0
MeanAngle = 0
VarianFlow = 0

BuffV = []
BuffA = []

ix = -1
iy = -1

qd = [0,0]

drawing = False
update = True
endDraw = False
fristplot = True
# Init All Value End #


def resize(img,per):
    scale_percent = per  # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    return cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

def draw_rectangle_with_drag(event, x, y, flags, param):
    global ix, iy, drawing, img, st, en, stL, enL, update

    if endDraw:pass

    if event == cv2.EVENT_LBUTTONDOWN:
        img = cv2.imread("plant1.jpg")
        ix = x
        iy = y
        st = (x, y)
        drawing = True
        print(st)

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True: en = (x, y)
        pass


    elif event == cv2.EVENT_LBUTTONUP:
        en = (x, y)
        stL = st
        enL = en
        drawing = False
        update = True
        print(en)


cv2.namedWindow(winname='Draw initial area')
cv2.setMouseCallback('Draw initial area',
                     draw_rectangle_with_drag)



def draw_flow(img, flow):
    global MeanVelocity,MeanAngle,VarianFlow ,node,qd

    h, w = img.shape[:2]
    step = (w/node)
    y, x = np.mgrid[step / 2:h:step, step / 2:w:step].reshape(2, -1).astype(int)
    fx, fy = flow[y, x].T

    lines = np.vstack([x, y, x - fx, y - fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)

    ynum = round(h/step)
    xnum = round(w/step)

    #print(f'node x,y : ({xnum},{ynum})')
    dt = int((xnum * (ynum-1))-xnum/1.15)

    #vector
    U = np.array([lines[t][0][0]-lines[t][1][0] for t in range(len(y))])
    V = np.array([lines[t][0][1]-lines[t][1][1] for t in range(len(y))])
    ang = []
    for i in range(len(V)):
        agcal = V[i] / U[i]
        er = ["-inf","inf","nan"]
        if str(agcal) in er :agcal = 0
        ang.append(agcal)

    qd[0] = np.mean(U)
    qd[1] = np.mean(V)
    mgnitue = (U**2 + V**2)**0.5
    angl = np.array([np.arctan(ag) for ag in ang])
    MeanAngle = np.mean(angl)* 57.2957795
    MeanVelocity = np.mean(mgnitue)
    VarianFlow = np.var(U)



    #print( stXYmn,' : ',edXYmn)

    img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(img_bgr, lines, 0, (0, 255, 0))

    for (x1, y1), (_x2, _y2) in lines:
        cv2.circle(img_bgr, (x1, y1), 1, (0, 255, 0), -1)

    #cv2.circle(img_bgr, (lines[dt][0][0],lines[dt][0][1]), 5, (255, 100, 200), -1)

    return img_bgr


def draw_hsv(flow):
    global MeanVelocity

    h, w = flow.shape[:2]
    fx, fy = flow[:, :, 0], flow[:, :, 1]

    act2 = np.arctan2(fy, fx)
    ang = act2
    v = np.sqrt(fx * fx + fy * fy)

    direct = act2 * 180 / np.pi

    MeanVelocity += np.mean(v)
    MeanVelocity = MeanVelocity/2

    hsv = np.zeros((h, w, 3), np.uint8)
    hsv[..., 0] = ang * (180 / np.pi / 2)
    hsv[..., 1] = 255
    hsv[..., 2] = np.minimum(v * 4, 255)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    return bgr

def get_VperMax():
    global BuffV,BuffA
    BuffV.append(MeanVelocity)

    try:
        perV = (MeanVelocity / max(BuffV)) * 100
    except:
        perV = 0
    return perV

def get_vectorUnit():
    return 0

def get_direction():
    global qd ,MeanAngle

    icons = ['right',
             'up',
             'left',
             'down']
    icon = 'w:direction-'
    deg = abs(MeanAngle)

    if qd[0] >= 0 and qd[1] >= 0:    # (+,+) quadrant 1
        deg = 0 + deg
        id = int(qd[0]<qd[1])
        icon += icons[id]
    elif qd[0] <= 0 and qd[1] >= 0:  # (-,+) quadrant 2
        deg = 180 - deg
        id = int(abs(qd[0]) > qd[1])
        icon += icons[id+1]
    elif qd[0] <= 0 and qd[1] <= 0:  # (-,-) quadrant 3
        deg = 180 + deg
        id = int(abs(qd[0]) < abs(qd[1]))
        icon += icons[id + 2]
    elif qd[0] >= 0 and qd[1] <= 0:  # (+,-) quadrant 4
        deg = 360 - deg
        id = int(qd[0] > abs(qd[1]))
        icon += icons[id - 1]

    return [int(deg),icon]

def Up2Adafruit(i):
    feedsname = ['velocity-percent','varian-flow','velocity','trend-direction']
    value = [get_VperMax(),VarianFlow,MeanVelocity,get_direction()[1]]

    #for i in range(k):
    feed = aio.feeds(feedsname[i])
    # aio.send_data(feed.key, value[i])
    aio.append(feed.key, value[i])

    print('\n---------status-------------')
    print(datetime.datetime.now().strftime("%d %b %Y %I:%M:%S%p"))
    print(f'\nVector mean : {qd[0]:.2f} i + {qd[1]:.2f} j')
    print(f'MeanAng : {MeanAngle:.2f}')

    print(f'\nMeanV : {MeanVelocity:.2f} pixel/s')
    print(f'perVmax : {get_VperMax():.2f} %')
    print(f'trend Direction : {get_direction()}')
    print(f'Var Flow : {VarianFlow:.2f}')


"""

# init_area()
while 1:
    suc, img = cap.read()
    img = cv2.flip(img, 1)

    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"{fps:.2f} FPS")

    if drawing or update:
        cv2.rectangle(img, pt1=st,
                      pt2=en,
                      color=(0, 255, 255),
                      thickness=1)

    if not suc:
        print('no video')
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        continue

    cv2.imshow('Draw initial area', img)

    key = cv2.waitKey(1)
    if key == ord('q') and update:
        #cv2.destroyWindow('Draw initial area')
        endDraw = True
        break

    time.sleep(1/fps)

"""

#initplot
y = np.linspace(0, 1, 50)
x = [0 for i in range(50)]
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111,polar=True)
line1, = ax.plot(x, y, 'b-')


# Main()
stUp = time.time()
k=0
while True:

    start = time.time()

    ## manage IMAGE
    suc, img = cap.read()
    try:
        img = resize(img,PER)
    except:
        img = prev

    if not suc:
        print('no video')
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        continue

    #img = cv2.flip(img,1)
    cv2.rectangle(img, pt1=st,
                  pt2=en,
                  color=(0, 255, 255),
                  thickness=1)

    cv2.imshow('Draw initial area', img)

    if update:
        try:
            prev = img[stL[1]:enL[1], stL[0]:enL[0]]
            prevgray = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY)
        except:
            prev = prev
            prevgray = prev
        BuffV = []
        BuffA = []
        update = False
        continue

    imgcal = img[stL[1]:enL[1], stL[0]:enL[0]]
    gray = cv2.cvtColor(imgcal, cv2.COLOR_BGR2GRAY)

    flow = cv2.calcOpticalFlowFarneback(prevgray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)

    prevgray = gray


    ## Upto Coud
    now = time.time()

    if now-stUp >= 10 and not drawing:
        Up2Adafruit(k)
        k+=1
        if k == 4: k=0
        stUp = time.time()

    ## Plot
    dg = get_direction()[0]
    dg = np.deg2rad(dg)
    x = [dg for i in range(50)]
    line1.set_xdata(x)
    fig.canvas.draw()
    fig.canvas.flush_events()


    text = [f'Velocity : {MeanVelocity:.2f} pixel/s',
            f'perVmax : {get_VperMax():.2f} %',
            f'trend Direction : {get_direction()}',
            f'Varian Flow : {VarianFlow:.2f}']
    hsv = draw_hsv(flow)
    for i in range(len(text)):
        cv2.putText(hsv, text[i], (10, 20+(30*i)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)


    cv2.imshow('flow', draw_flow(gray, flow))
    cv2.imshow('flow HSV', hsv)

    imgAll = np.concatenate((draw_flow(gray, flow), draw_hsv(flow)), axis=0)
    cv2.imshow('Overall', imgAll)

    perfps = (imgcal.shape[0]*imgcal.shape[1])/(img.shape[0]*img.shape[1])
    t = 0.5
    delay = (perfps/t-1)**2 if perfps <= t else 0

    end = time.time()
    delay = delay*(end-start)
    time.sleep(delay)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

#sample.mp4
cap.release()
cv2.destroyAllWindows()
