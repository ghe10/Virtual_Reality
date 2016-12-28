import viz,vizact,vizdlg,viztask,vizshape
import math,random
import testSensorNew
#import oculus_08 as oculus
#hmd= oculus.Rift()
#viz.link(hmd.getSensor(), viz.MainView)

viz.setMultiSample(4)
viz.fov(60)
viz.go()
viz.collision(viz.ON)
viz.gravity(10)
viz.phys.enable()

#Add message panel and audio
message_panel = vizdlg.MessagePanel('')
message_panel.setPanelScale(2.0)
message_panel.setScreenAlignment(viz.ALIGN_CENTER_TOP)
message_panel.visible(True)
message_panel.setText('Welcome to VR paddle boarding experience!')
audio = viz.addAudio('thatGoodShip.mp3')
audio.play()

class PlayerPrefs:
    def __init__(self, is_right_handed, height):
        self.is_right_handed = is_right_handed
        self.height = height
        

    # PUBLIC

    def paddle_scale(self):
        # Scale paddle to 3/4 height
        s = 2*self.height
        return [s, s, s]

    def board_scale(self):
        # Scale board to 1.5x height
        s = 1.5*self.height
        return [s, s, s]

    def paddle_position(self):
        # Right handed players will see paddle on their left
        return [self.height*0.1 * (-1 if self.is_right_handed else 1), -self.height/2.0, self.height*0.15]

    def board_position(self):
        bs = self.board_scale()
        pos=[bs[0]/2.0-0.25, -self.height + 0.2, -0.75*bs[2]]
        return pos

class Player:
    def __init__(self, camera, player_prefs, move_speed=5.0, turn_speed=25.0):
        self.view = viz.MainView
        self.camera = camera
        self.player_prefs = player_prefs
        # TODO: Init paddle and board based on player_prefs
        self.paddle = None
        self.board  = viz.addChild('seaeagle.dae')
        self.board.collidePlane()
        self.board.disable(viz.DYNAMICS)
        self.move_speed = move_speed
        self.turn_speed = turn_speed
        self.max_speed = 10
        
        self.ball_dir = viz.add('white_ball.wrl')
        #viz.link(self.board, self.ball_board)
        
        
        self.PS1 = None
        self.PS2 = None
        self.ball1 = None
        self.ball = None
        self.last_pos_ball = None
        self.water_level = 0.4

    # PUBLIC
    def setUpPaddle(self):
        PS1 = testSensorNew.addPhaseSpaceMulti()
        PS1.addMarkers(0, 1, 2, 3, 4, 5, 6)
        PS1.setPointMarker()
        PS1.setServerAddress('128.174.14.227')
        PS1.setFrequency(240)
        PS1.setScale(0.001, 0.001, 0.001)
        PS1.setSlave()
            #PS1.startStreaming()
        

        viz.setOption('viz.model.apply_collada_scale',1)
        
        
        ball = viz.add('white_ball.wrl')
        ball.color(viz.WHITE)
        #ball.setScale(0)
        #ojo = viz.addChild('sky_day.osgb')
        print PS1
        viz.link(PS1, ball)

        PS2 = testSensorNew.addPhaseSpaceMulti()
        PS2.addMarkers(7, 8, 9, 10, 11, 12)
        PS2.setPointMarker()

        PS2.setServerAddress('128.174.14.227')
        PS2.setFrequency(240)
        PS2.setScale(0.001, 0.001, 0.001)
        PS2.setSlave()
        PS1.startStreaming()
        PS2.startStreaming()
        
        ball1 = viz.add('white_ball.wrl')
        ball1.color(viz.WHITE)
        #ball1.alpha = 0
        #ball1.setScale(0)
        #ball1.setPosition(0,2,0)
        
        print PS2
        viz.link(PS2, ball1)


        water_level = 0
        
        self.paddle = viz.addChild('paddle/newOar.dae', parent = self.board)
        #self.paddle.collideMesh()
        #self.paddle.disable(viz.DYNAMICS)
        self.paddle.setEuler( [ 90, 0, 0 ] )
        #viz.link(ball, paddle)
        v_x = [1, 0, 0]
        global v_paddle 
        v_paddle_last = self.paddle.getEuler()
        Trans = vizmat.Transform()
        
        '''
        global prev_p0 
        prev_p0 = ball.getPosition()
        global prev_p1
        prev_p1 = ball1.getPosition()
        '''
        self.last_pos_ball = ball.getPosition()
        
        def TestRecording():
            print ball.getPosition()
            print ball1.getPosition()

        def RotatePaddle():
            p0 = ball.getPosition();
            p1 = ball1.getPosition();
            #camera_pos = self.camera.getPosition()
            board_pos = self.camera.getPosition()
            #camera_rot = self.camera.get()
            board_dir= self.camera.getEuler()
            #print("dir" , board_dir)
            add_x = math.cos(board_dir[2])
            add_z = math.sin(board_dir[2])
            #print("dir", add_x)
            v_true = [p1[0] - p0[0],p1[1] - p0[1],p1[2] - p0[2] ]
            position_true = [(p1[0] + p0[0])/2 - 0.2,(p1[1] + p0[1])/2 - 1,(p1[2] + p0[2])/2 + 1.5]
            
            position_true[1] -= 2
            Trans.makeVecRotVec(v_x, v_true)
            euler = Trans.getEuler()
            #for data in euler:
            #    data = -data
            #qut = 
            self.paddle.setEuler(euler)
            #self.paddle.setPosition(self.view.getPosition())
            #self.paddle.setEuler(self.view.getEuler())
            #self.paddle.setPosition([1,0,0], viz.REL_LOCAL)
            self.paddle.setPosition(position_true)

        
        def updateSpeed():
            p0 = ball.getPosition();
            p1 = ball1.getPosition();
            mid = (p0[0] + p0[1]) / 2
            camera_pos = self.camera.getPosition()
            print("paddle height = ", p0[1])
            move_amount = 1
            turn_amount = 0
            if(self.last_pos_ball[1] > p0[1] + 0.05 and p0[1] <= self.water_level):
                message_panel.setText('Let\'s explore the islands!')
                self.move_speed = min(self.move_speed + 2, self.max_speed)
                print("speed up" , self.move_speed)
            elif(p0[1] <= self.water_level and abs(self.last_pos_ball[1] - p0[1]) < 1):
                self.move_speed = max(self.move_speed - 0.1, 2)
                print("slow down" , self.move_speed)
            else:
                self.move_speed = max(self.move_speed - 0.01, 2)
                print("no action" , self.move_speed)
            self.last_pos_ball[1] = p0[1]
            self.transform(move_amount, turn_amount)
            #print(Trans.getForward(self.camera.getEuler()))
            #print(self.camera.getEuler().getForward())
            #print("ball pos", self.ball_board.getPosition)
            #print("board pos", self.board.getPosition())
        
        def updateDirection():
            p0 = ball.getPosition();
            p1 = ball1.getPosition();
            mid = (p0[0] + p0[1]) / 2
            camera_pos = self.camera.getPosition()
            move_amount = 0
            turn_amount = 0
            print("mid", mid)
            print("p0[0]", p0[0])
            if(self.last_pos_ball[1] > p0[1] + 0.05 and p0[1] <= self.water_level):
                if(p0[0] > mid):
                    #turn right    
                    print("turn right")
                    turn_amount = 10
                else:
                    print("turn left")
                    turn_amount = -10
            '''
            elif(p0[1] <= self.water_level and abs(self.last_pos_ball[1] - p0[1]) < 0.05):
                if(p0[0] < mid):
                    #turn left   
                    print("turn left")
                    turn_amount = -1
                else:
                    print("turn right")
                    turn_amount = 1
            '''
            self.transform(move_amount, turn_amount)
        
        def updatePos():
            move_amount = 1
            turn_amount = 0
            self.transform(move_amount, turn_amount)
                
        vizact.onkeydown(' ', TestRecording)
        vizact.ontimer(0.0, RotatePaddle)
        vizact.ontimer(0.0, updateDirection)
        vizact.ontimer(0.4, updateSpeed)
        vizact.ontimer(0.0, updatePos)
        self.PS1 = PS1
        self.PS2 = PS2
        self.ball1 = ball1
        self.ball = ball

    
   

    def transform(self, move_amount, turn_amount):
        self._move(move_amount)
        self._turn(turn_amount)
        if self.paddle is not None:
            self._update_paddle()
        if self.board is not None:
            self._update_board()

    # PRIVATE

    def _move(self, amount):
        self.camera.move([0, 0, amount*self.move_speed*viz.elapsed()], viz.BODY_ORI)

    def _turn(self, amount):
        self.camera.setEuler([amount*self.turn_speed*viz.elapsed(), 0, 0], viz.BODY_ORI, viz.REL_PARENT)

    def _update_paddle(self):
        a=0
        #self.paddle.setPosition(self.camera.getPosition())
        #self.paddle.setEuler(self.camera.getEuler(viz.BODY_ORI))
        #self.paddle.setPosition(self.player_prefs.paddle_position(), viz.REL_LOCAL)

    def _update_board(self):
        self.board.setPosition(self.camera.getPosition())
        self.board.setEuler(self.camera.getEuler(viz.BODY_ORI))
        self.board.setPosition(self.player_prefs.board_position(), viz.REL_LOCAL)

#SETTING SKYBOX
env = viz.addEnvironmentMap('sky.jpg')
sky = viz.addCustomNode('skydome.dlc')
sky.texture(env)

#def path_func(complexity):
 #   W = [random.random() for i in range(int(complexity))]
  #  print(W)
   # def f(x):
    #    y = sum([math.sin(w*x/5) for w in W])
     #   return y / complexity
   # return f

#f = path_func(50.0)

land=viz.addChild('Land/model.osgb')
land.setPosition([0,1,0])

male = viz.addAvatar('vcc_male.cfg', pos=(-38,5,40), euler=(90,0,0) )
male.setScale(1,1,1)
male.state(5)

female = viz.addAvatar('vcc_female.cfg', pos=(-35,4.75,40), euler=(270,0,0) )
female.setScale(1,1,1)
female.state(5)

basket=viz.addChild('basketball.osgb')
basket.setPosition([-140,2.8,130])

male1 = viz.addAvatar('vcc_male.cfg', pos=(-140,2.8,130), euler=(90,0,0) )
male1.setScale(2,2,2)
male1.state(5)

female1 = viz.addAvatar('vcc_female.cfg', pos=(-135,2.8,130), euler=(270,0,0) )
female1.setScale(2,2,2)
female1.state(5)

for x in xrange(0,10,2):
    for z in [-4,4]:
       tree = viz.addChild('tree/palmtree.dae',cache=viz.CACHE_CLONE)
       tree.setPosition([x-30,3,z+40])
       
for x in range(0,5):
    bird = viz.addAvatar('pigeon.cfg',pos=(x-30,4-x*0.1,37))
    bird.collideCapsule()
    bird.disable(viz.DYNAMICS)
    bird.state(3)
    bird.setScale(1,1,1)
    
#corner 1
ocean = viz.add('Ocean/Ocean.osgb')
ocean2=viz.add('Ocean/Ocean.osgb')
ocean3=viz.add('Ocean/Ocean.osgb')

ocean.setScale(100,25,100)
ocean2.setScale(100,32,100)
ocean3.setScale(100,20,100)

spin = vizact.spin(0, 1, 0, 25, viz.FOREVER)
spin2 = vizact.spin(0, -1, 0, 30, viz.FOREVER)
spin3 = vizact.spin(0, 1, 0, 5, viz.FOREVER)

ocean.addAction(spin)
ocean2.addAction(spin2)
ocean3.addAction(spin3)

player_prefs = PlayerPrefs(True, 1.0)
player = Player(viz.MainView, player_prefs)
player.setUpPaddle()

'''
def update():
    move_amount = 2
    turn_amount = 0
    
    if viz.key.isDown(viz.KEY_UP):
        move_amount *= 10
        message_panel.setText('Let\'s explore the islands!')
    elif viz.key.isDown(viz.KEY_DOWN):
        move_amount *= -1.1
    if viz.key.isDown(viz.KEY_LEFT):
        turn_amount = -1.0
    elif viz.key.isDown(viz.KEY_RIGHT):
        turn_amount = 1.0
    player.transform(move_amount, turn_amount)

vizact.ontimer(0, update)
'''