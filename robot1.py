import utils
from utils import Position as Position
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot1(RCJSoccerRobot):

    def run(self):
        self.transmitter = utils.Transmitter(self)
        self.ball_log = utils.Manage_log(200)
        self.robot_log = utils.Manage_log(20)
        self.move = utils.Move(self.robot_log, self.ball_log)
        self.mode = "goalkeeper"  # or captain
        self.init = False
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                teamData = self.transmitter.getData()
                gps_data = self.get_gps_coordinates()
                position = Position(
                    gps_data[0], gps_data[1], self.get_compass_heading())

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                    my_ball_pos = utils.get_ballpos(
                        position, ball_data["direction"][1], ball_data["direction"][0], ball_data["strength"])
                else:
                    ball_data = None
                    my_ball_pos = None

                ball_pos = my_ball_pos

                if (self.init == False):
                    """
                    teamcolor: -1  blue
                    1  yellow
                    """
                    if (position.y >= 0):
                        teamcolor = -1
                    else:
                        teamcolor = 1
                    self.init = True

                if ball_pos == None:
                    for i in teamData.values():
                        if i["ballpos"] != None:
                            ball_pos = i["ballpos"]

                self.ball_log.set_data(ball_pos)
                if self.mode == "goalkeeper":
                    motorOut = self.move.goalkeeper(
                        position, self.ball_log.get_PrePos(), teamcolor)
                elif self.mode == "captain":
                    motorOut = self.move.captain(
                        position, self.ball_log.get_PrePos(), teamcolor)

                self.left_motor.setVelocity(motorOut[0])
                self.right_motor.setVelocity(motorOut[1])

                self.transmitter.sendData(
                    self.player_id, position, my_ball_pos)
