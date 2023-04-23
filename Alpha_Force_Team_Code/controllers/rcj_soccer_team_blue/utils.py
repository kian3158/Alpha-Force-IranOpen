from rcj_soccer_robot import RCJSoccerRobot as sr
import numpy as np
import math
from collections import deque


class Position:
    def __init__(self, x: float, y: float, dir: float = 0):
        self.x = x
        self.y = y
        self.dir = dir

    def __repr__(self):
        return f'x:{self.x}  y:{self.y}  dir:{self.dir}'

    def __str__(self):
        return f'x:{self.x}  y:{self.y}  dir:{self.dir}'


BLUE_GOAL_POSITION_UPPER = Position(0.2, 0.745)
BLUE_GOAL_POSITION_CENTER = Position(0, 0.745)
BLUE_GOAL_POSITION_LOWER = Position(-0.2, 0.745)


YELLOW_GOAL_POSITION_UPPER = Position(0.2, -0.745)
YELLOW_GOAL_POSITION_CENTER = Position(0, -0.745)
YELLOW_GOAL_POSITION_LOWER = Position(-0.2, -0.745)


def get_distance(posa: Position, posb: Position = Position(0, 0)) -> float:
    """
    Calculates the Euclidean dist between two positions in the soccer field.
    It takes two parameters,
    posa and posb, which are instances of the Position class,
    and returns the dist between them as a floating-point number.
    """
    return ((posa.x - posb.x)**2 + (posa.y - posb.y)**2)**0.5


def get_direction(ball_vector: float) -> int:
    """
    Takes in the current vector of the ball
    with respect to the robot and returns an integer
    representing the direction the robot should face to face the ball.
    """
    if -0.13 <= ball_vector <= 0.13:
        return 0
    return -1 if ball_vector < 0 else 1


def get_ballpos(r_pos: Position, ball_dirs: float, ball_dirc: float, ball_str: float) -> Position:
    """
    Takes in the position and direction of the robot,
    as well as the direction and strength of the ball,
    and returns the expected position of the ball based on the robot's movement.
    """
    sin_dir = np.arcsin(ball_dirs)
    cos_dir = np.arccos(ball_dirc)
    cos_dir -= np.pi / 2
    new_dir = np.abs(sin_dir)
    if sin_dir < 0 and cos_dir > 0:
        new_dir = np.pi + new_dir
    elif sin_dir < 0:
        new_dir = np.pi * 2 - new_dir
    elif cos_dir > 0:
        new_dir = np.pi - new_dir
    dir = change_dir(r_pos.dir + new_dir)
    x = r_pos.x - np.sin(dir) * (ball_str ** -0.50813)
    y = r_pos.y + np.cos(dir) * (ball_str ** -0.50813)
    ret_pos = Position(x, y, 0)
    return ret_pos


def get_motorOut(current_pos: Position, new_pos: Position) -> tuple:
    """
    Takes in the current position of the robot
    and the position it needs to move to
    and returns the motor outputs required to move
    the robot to the desired position.
    """

    dir = math.atan2(new_pos.x - current_pos.x, new_pos.y - current_pos.y)
    dir = change_dir(dir + current_pos.dir)
    left_motor = 10
    right_motor = 10

    if np.sin(dir) > 0:
        if np.cos(dir) > 0:
            right_motor -= max(0, abs(dir) * 12)

        elif np.cos(dir) < 0:
            left_motor *= -1
            right_motor *= -1
            right_motor += max(0, (math.pi - abs(dir)) * 12)

    elif np.sin(dir) < 0:
        if np.cos(dir) > 0:
            left_motor -= max(0, abs(dir) * 12)

        elif np.cos(dir) < 0:
            left_motor *= -1
            right_motor *= -1
            left_motor += max(0, (math.pi - abs(dir)) * 12)

    return right_motor, left_motor


def change_dir(dir):
    """
    Converts an angle to the range of -π to π.
    """
    if dir > np.pi:
        dir -= 2 * np.pi
    if dir < -np.pi:
        dir += 2 * np.pi
    return dir


class Transmitter:
    def __init__(self, robot: sr):
        self.robo = robot

    def isdata(self) -> bool:
        # Checks if there is new team data available and returns a boolean value accordingly.
        return self.robo.is_new_team_data()

    def getData(self) -> dict:
        """
        Reads and returns the team data that has been received since the last time getData was called.
        It loops through all the data using the isdata method and extracts the robot's position
        and the ball's position if available.
        It then returns a dictionary with the robot ID as the key and a dictionary of the robot's position
        and the ball's position (if available) as the value.
        """
        team_data: dict = {}
        while self.isdata():
            data = self.robo.get_new_team_data()
            position_data = Position(
                data["pos_x"], data["pos_y"], data["pos_dir"])
            if (data["is_ball_data"]):
                ball_pos = Position(data["ball_x"], data["ball_y"], 0)
            else:
                ball_pos = None
            team_member_position = {
                "robot_pos": position_data,
                "ballpos": ball_pos
            }
            team_data[data["robot_id"]] = team_member_position
        return team_data

    def sendData(self, robot_id: int, robot_pos: Position, ballpos: Position = None):
        """It takes the robotid, robotpos (a Position object),
        and ballpos (also a Position object, but optional) as arguments.
        The method then sends this data to the robot using the send_data_to_team method.
        If ballpos is None, the method sends data indicating that the ball is not visible,
        and if ballpos is not None,
        the method sends data indicating the ball's position
        """
        if ballpos == None:
            data = [robot_id, robot_pos.x, robot_pos.y,
                    robot_pos.dir, False, -100, -100]
        else:
            data = [robot_id, robot_pos.x, robot_pos.y,
                    robot_pos.dir, True, ballpos.x, ballpos.y]
        self.robo.send_data_to_team(data)


class Manage_log:

    def __init__(self, length: int):
        # Initializes the positionLog deque with a given maximum length.
        self.positionLog = deque(maxlen=length)

    def set_data(self, data: Position):
        # Appends the given Position data to the deque.
        self.positionLog.append(data)

    def is_seen(self) -> bool:
        # Returns True if there is any non-None data in the deque.
        for i in self.positionLog:
            if i != None:
                return True
        return False

    def get_Vector(self) -> Position:
        # Calculates the direction vector of the ball using the first and last non-None data in the deque.
        start_pos = None
        final_pos = None
        for i in self.positionLog:
            if i != None:
                if start_pos == None:
                    start_pos = i
                else:
                    final_pos = i
        if start_pos == None:
            return Position(0, 0)
        if start_pos != None and final_pos == None:
            return Position(0, 0)
        dist = get_distance(start_pos, final_pos)
        if dist <= 0.00001:
            return Position(0, 0)
        return Position((final_pos.x - start_pos.x) / dist, (final_pos.y - start_pos.y) / dist)

    def get_Speed(self) -> Position:
        # Calculates the average speed of the ball using the last length non-None data in the deque.
        start_pos = None
        final_pos = None
        count = 1
        consecutive_count = 1
        for i in range(1, len(list(self.positionLog))):
            if self.positionLog[-i] != None:
                if start_pos == None:
                    start_pos = self.positionLog[-i]
                else:
                    final_pos = self.positionLog[-i]
                    count += consecutive_count
                    consecutive_count = 1
            elif start_pos != None:
                consecutive_count += 1
            if i > 6 and final_pos != None:
                break
        return Position(-(final_pos.x - start_pos.x) / count, -(final_pos.y - start_pos.y) / count) if start_pos != None and final_pos != None else None

    def get_PrePos(self) -> Position:
        # Calculates the previous position of the ball using the current position and average speed.
        start_pos = None
        count = 1
        if self.positionLog[-1] != None:
            return self.positionLog[-1]
        for i in list(self.positionLog)[-1::-1]:
            if i != None:
                start_pos = i
                break
            else:
                count += 1
        speed = self.get_Speed()
        if speed == None:
            speed = Position(0, 0)
        return Position(start_pos.x + speed.x * count, start_pos.y + speed.y * count) if start_pos != None else None

    def is_stop(self, flame: int) -> bool:
        # Returns True if the ball has stopped for a given number of frames.
        start_pos = None
        final_pos = None
        for i in list(self.positionLog)[-1:-flame:-1]:
            if i != None:
                if start_pos == None:
                    start_pos = i
                else:
                    final_pos = i
        return True if start_pos != None and final_pos != None and get_distance(start_pos, final_pos) < 0.075 else False

    def is_respawn(self) -> bool:
        # Returns True if the ball has recently respawned.
        if self.positionLog[-1] == None:
            return True if self.is_stop(100) else False
        else:
            start_pos = self.positionLog[-1]
            final_pos = None
            for i in self.positionLog[-2::-1]:
                if i != None:
                    final_pos = i
                    break
        return False if final_pos == False else False if get_distance(start_pos, final_pos) < 0.1 else True


class Move:
    offense_alphaflag = False
    offense_alphaflag = False
    at_alphaflag = False
    at_forceflag = False
    alpha = 0
    robot_log: Manage_log = None
    ball_log: Manage_log = None

    def __init__(self, robot_log, ball_log):
        self.robot_log = robot_log
        self.ball_log = ball_log

    def calculate_movement(self, current_pos: Position, new_pos: Position) -> tuple:
        """
        calculates and returns the motor outputs (right, left) 
        to make the robot move smoothly from the current position (current_pos) 
        to the new position (new_pos). 
        The robot's direction is changed according to the dir parameter, 
        which is calculated by taking the difference between the new position 
        and the current position and adjusting the robot's direction to it. 
        The change_dir function is called to normalize the direction value. 
        Depending on the direction, 
        the motor outputs are adjusted accordingly 
        to make the robot move towards the new position.
        """
        dir = math.atan2(new_pos.x - current_pos.x, new_pos.y - current_pos.y)
        dir = change_dir(dir + current_pos.dir)

        left_motor = 10
        right_motor = 10

        if (np.sin(dir) > 0.1 and np.cos(dir) < 0) or (np.sin(dir) < -0.1 and np.cos(dir) > 0):
            left_motor *= -1
        elif (np.sin(dir) < -0.1 and np.cos(dir) < 0) or (np.sin(dir) > 0.1 and np.cos(dir) > 0):
            right_motor *= -1
        elif np.cos(dir) < 0:
            left_motor *= -1
            right_motor *= -1
        return right_motor, left_motor

    goalkeeper_goto = None
    goalkeeper_goto2 = None
    goalkeeper_shaking = False

    def goalkeeper(self, robot_pos: Position, ball_x: Position, teamColor: int) -> tuple:
        """
       Calculates and returns the motor outputs (right, left) for the goalkeeper to make the robot move towards 
       a designated position (goalkeeper_got) or to stop moving (goalkeeper_shaking is True). 
       The goalkeeper's position depends on the ball's position, 
       and the logic changes according to the ball's position relative to the goalpost. 
       If the ball is near the goalkeeper, it stays still and waits. 
       If the ball is in the goal area, it moves to a position to protect the goal. 
       If the ball is in other areas, it moves to an appropriate position to intercept the ball. 
       Additionally, there are two flags (goalkeeper_shaking and goalkeeper_goto2) 
       to prevent the goalkeeper from getting stuck in a respawn point or to simulate a quivering movement. 
       If goalkeeper_shaking is True, 
       the robot moves towards goalkeeper_goto2 instead of goalkeeper_got.
        """

        if ball_x != None:
            target_point_x = self.ball_log.get_Vector().x * (-0.01 * teamColor - ball_x.y) / \
                max(0.01, abs(self.ball_log.get_Vector().y))
            if ball_x.y * teamColor * -1 < 0.3 or (abs(ball_x.x) < 0.3 and ball_x.y * teamColor * -1 < 0.65):
                goto = Position(
                    min(0.3, max(-0.3, ball_x.x)), -teamColor * 0.55, 0)
            elif ball_x.y * teamColor * -1 < 0.65:
                goto = Position(
                    min(0.25, max(-0.2, ball_x.x)), -teamColor * 0.70, 0)
            elif self.ball_log.positionLog[-1] == None:
                goto = Position(0, -teamColor * 0.70)
            else:
                goto = Position(ball_x.x / abs(ball_x.x), -teamColor * 0.70, 0)

            if get_distance(ball_x, robot_pos) < 0.1 and abs(ball_x.y) < abs(robot_pos.y) and abs(ball_x.x) > 0.35:
                goto = ball_x
        else:
            goto = Position(0, -teamColor * 0.55, math.pi / 2)

        self.goalkeeper_goto = goto

        if get_distance(robot_pos, self.goalkeeper_goto) < 0.01 and self.goalkeeper_shaking == False and (ball_x == None or (ball_x != None and ball_x.y / abs(ball_x.y) / teamColor > 0)):

            self.goalkeeper_shaking = True
            self.goalkeeper_goto2 = Position(self.goalkeeper_goto.x - self.goalkeeper_goto.x / max(
                0.01, abs(self.goalkeeper_goto.x)) * 0.1, self.goalkeeper_goto.y)
        if self.goalkeeper_shaking and get_distance(robot_pos, self.goalkeeper_goto2) < 0.01:
            self.goalkeeper_shaking = False

        if self.ball_log.is_stop(100) and ball_x != None and ball_x.y / abs(ball_x.y) / teamColor > 0:
            self.goalkeeper_shaking = False
            self.goalkeeper_goto = Position(0, -0.3 * teamColor)
        if self.ball_log.is_stop(100) and self.robot_log.is_stop(100):
            self.goalkeeper_shaking = False
            self.goalkeeper_goto = Position(0, -teamColor * 0.55, math.pi / 2)

        if not self.goalkeeper_shaking:
            return self.calculate_movement(robot_pos, self.goalkeeper_goto)
        else:
            return self.calculate_movement(robot_pos, self.goalkeeper_goto2)

    def captain(self, robot_pos: Position, ball_x: Position, teamcolor: int) -> tuple:
        """
        Calculates and returns the motor outputs (right, left) for the captain to move the robot towards the goalpost of the opposing team. 
        The captain always moves around the ball and tries to find a way to move forward. 
        There are several flags (offense_alphaflag, offense_alphafla2, and at_alphaflag) 
        to control the robot's turning behavior while moving around the ball. 
        The robot moves towards the target goalpost 
        while keeping the ball in the center of the field of view.
        """

        def in_range(dir1, dir2, dif) -> bool:
            min_diff = abs((dir1 - dir2) % (math.pi * 2))
            max_diff = math.pi * 2 - min_diff
            return min(min_diff, max_diff) <= dif

        if ball_x == None:
            return get_motorOut(robot_pos, Position(0, -teamcolor * 0.3))

        current_dir = math.atan2(
            ball_x.x - robot_pos.x, ball_x.y - robot_pos.y)

        future_dir = math.atan2(-ball_x.x,
                                BLUE_GOAL_POSITION_CENTER.y * teamcolor - ball_x.y)

        different_dir = math.pi / 3

        if in_range(future_dir, current_dir, different_dir):

            move_right = get_distance(robot_pos, ball_x)

            next_pos = Position(
                ball_x.x + move_right * math.sin(future_dir),
                ball_x.y + move_right * math.cos(future_dir)
            )
            return get_motorOut(robot_pos, next_pos)

        if get_distance(robot_pos, ball_x) >= 0.4:
            return get_motorOut(robot_pos, Position(ball_x.x - np.sign(ball_x.x) * 0.2, ball_x.y - teamcolor * 0.3))

        if teamcolor * ball_x.y < -0.4:
            return self.goalkeeper(robot_pos, ball_x, teamcolor)

        vector = self.robot_log.get_Vector()

        robot_gotoDir = math.atan2(vector.x, vector.y)

        rotate = -teamcolor * np.sign(math.sin(current_dir))
        if in_range(math.pi, robot_gotoDir, math.pi / 24):
            rotate = - teamcolor * np.sign(vector.x)

        move_right = max(0, get_distance(robot_pos, ball_x) - 0.02)

        current_dir = change_dir(current_dir + math.pi)

        next_pos = Position(
            ball_x.x + move_right
            * math.sin(current_dir + rotate * math.pi / 36),
            ball_x.y + move_right
            * math.cos(current_dir + rotate * math.pi / 36)
        )

        return get_motorOut(robot_pos, next_pos)

    def offense(self, robot_pos: Position, ball_x: Position, teamcolor: int) -> tuple:
        """
        first checks if the ball is present or not. If it is not present, 
        the robot moves towards the center of the field. 
        If the ball is present, 
        the robot moves towards the opponent's goal with the ball.
        """
        if (teamcolor == -1):
            if ball_x == None:
                proximity = ((0 - robot_pos.x)**2 + (0 - robot_pos.y)**2)**0.5
                if (proximity > 0.01):
                    return_data = get_motorOut(robot_pos, Position(0, 0))
                else:
                    return_data = (10, -10)
                self.alpha = 100
            else:
                if (ball_x.y > robot_pos.y or self.at_alphaflag == True):
                    self.at_alphaflag = True
                    if (ball_x.x > 0):
                        if (robot_pos.x > ball_x.x or self.at_forceflag == True):
                            self.at_forceflag = True
                            return_data = get_motorOut(
                                robot_pos, Position(-0.7, ball_x.y - 0.15))
                            self.alpha = 1
                            if ball_x.x - 0.25 > robot_pos.x:
                                self.at_forceflag = False
                        else:
                            return_data = get_motorOut(robot_pos, Position(
                                ball_x.x - 0.15, ball_x.y + 0.15))
                            self.alpha = 2
                    else:
                        if (ball_x.x > robot_pos.x or self.at_forceflag == True):
                            self.at_forceflag = True
                            return_data = get_motorOut(
                                robot_pos, Position(0.7, ball_x.y - 0.15))
                            self.alpha = 3
                            if (robot_pos.x > ball_x.x - 0.25):
                                self.at_forceflag = False
                        else:
                            return_data = get_motorOut(robot_pos, Position(
                                ball_x.x + 0.15, ball_x.y + 0.15))
                            self.alpha = 4
                    if (robot_pos.y > ball_x.y + 0.1):
                        self.at_alphaflag = False
                        self.at_forceflag = False
                else:
                    ball_proximity = get_distance(robot_pos, ball_x)
                    if (ball_proximity > 0.1):
                        shoot_a = (ball_x.x - YELLOW_GOAL_POSITION_CENTER.x) / \
                            (ball_x.y - YELLOW_GOAL_POSITION_CENTER.y)
                        shoot_y = ball_x.y + 0.05
                        shoot_x = shoot_a * (shoot_y - ball_x.y) + ball_x.x
                        return_data = get_motorOut(
                            robot_pos, Position(shoot_x, shoot_y))

                    else:
                        return_data = get_motorOut(robot_pos, ball_x)
                    self.alpha = 10
        else:
            if ball_x == None:
                proximity = ((0 - robot_pos.x)**2 + (0 - robot_pos.y)**2)**0.5
                if (proximity > 0.01):
                    return_data = get_motorOut(robot_pos, Position(0, 0))
                else:
                    return_data = (10, -10)
                self.alpha = 100
            else:
                if (ball_x.y < robot_pos.y or self.at_alphaflag == True):
                    self.at_alphaflag = True
                    if (ball_x.x > 0):
                        if (robot_pos.x > ball_x.x or self.at_forceflag == True):
                            self.at_forceflag = True
                            return_data = get_motorOut(
                                robot_pos, Position(-0.7, ball_x.y + 0.15))
                            self.alpha = 1
                            if ball_x.x - 0.25 > robot_pos.x:
                                self.at_forceflag = False
                        else:
                            return_data = get_motorOut(robot_pos, Position(
                                ball_x.x - 0.15, ball_x.y - 0.15))
                            self.alpha = 2
                    else:
                        if (ball_x.x > robot_pos.x or self.at_forceflag == True):
                            self.at_forceflag = True
                            return_data = get_motorOut(
                                robot_pos, Position(0.7, ball_x.y + 0.15))
                            self.alpha = 3
                            if (robot_pos.x > ball_x.x + 0.25):
                                self.at_forceflag = False
                        else:
                            return_data = get_motorOut(robot_pos, Position(
                                ball_x.x + 0.15, ball_x.y - 0.15))
                            self.alpha = 4
                    if (robot_pos.y < ball_x.y - 0.05):
                        self.at_alphaflag = False
                        self.at_forceflag = False
                else:
                    ball_proximity = get_distance(robot_pos, ball_x)
                    if (ball_proximity > 0.1):

                        shoot_a = (ball_x.x - BLUE_GOAL_POSITION_CENTER.x) / \
                            (ball_x.y - BLUE_GOAL_POSITION_CENTER.y)
                        shoot_y = ball_x.y - 0.05
                        shoot_x = shoot_a * (shoot_y - ball_x.y) + ball_x.x
                        return_data = get_motorOut(
                            robot_pos, Position(shoot_x, shoot_y))

                    else:
                        return_data = get_motorOut(robot_pos, ball_x)
                        self.alpha = 10
        return return_data