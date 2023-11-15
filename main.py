from niryoCH import *

robot = NiryoRobot("10.10.101.36")

robot.arm.calibrate_auto()
robot.tool.update_tool()
robot.tool.release_with_tool()

hauteur_pose_initiale_sol = 20 # 20 cm à mesurer à l'ecole

#hauteur_pose_initiale_sol = mesure_hauteur_pince_sol(robot, .....)



###########################################################################################

def depalcement (robot, axe, cm):
    if axe == "x":
        robot.arm.shift_pose(RobotAxis.X, cm/100)
    elif axe == "y":
        robot.arm.shift_pose(RobotAxis.Y, cm / 100)
    elif axe == "z":
        robot.arm.shift_pose(RobotAxis.Z, cm / 100)




robot.arm.move_joints([0.0,0.0,0.0,0.0,-1.57,0.0])

continuer = True
while continuer:
    axe = input("axe et distance : ")

    if axe != "f":
        rslt = axe.split(',')
        depalcement(robot, rslt[0], int(rslt[1]))
    else:
        continuer = False


###########################################################################################

input("Appuyer une touche pour continuer : ")
robot.arm.move_to_home_pose()
robot.end()