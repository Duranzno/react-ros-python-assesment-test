import { MAX_SIDE_ROS, MAX_SIDE_WEB } from "../constants"
import { Pose } from "./turtlesim/models"
import { Pose2D } from "./go.actionlib.model"
import { RotablePoint } from "./extra.models"

export const fromPixiToTurtlesim = ({ x: pixiX, y: pixiY, theta }: RotablePoint): Pose => {
    //To transform the coordinates we proportionate the coordinates
    const x = (pixiX * MAX_SIDE_ROS) / MAX_SIDE_WEB
    let y = (pixiY * MAX_SIDE_ROS) / MAX_SIDE_WEB
    //as both maps have different y axis orientations we inverse the value of them 
    y = Math.abs(MAX_SIDE_ROS - y)
    return ({ x, y, theta, "angular_velocity": 0, "linear_velocity": 0 })
}
export const fixPixiTheta = (theta: number) => theta - (Math.PI / 2);
export const fromTurtlesimToPixi = ({ x: turtleX, y: turtleY, theta: turtleTheta }: Pose | Pose2D): RotablePoint => {
    const x = (Number(turtleX.toFixed(4)) * MAX_SIDE_WEB) / MAX_SIDE_ROS
    //FIXME: The Theta Angle of PIXI is different to the one ROS manages
    const theta = fixPixiTheta(turtleTheta)
    let y = (Number(turtleY.toFixed(4)) * MAX_SIDE_WEB) / MAX_SIDE_ROS
    //as both maps have different y axis orientations we inverse the value of them 
    y = Number(Math.abs(MAX_SIDE_WEB - y).toFixed(4))
    return { x, y, theta }
}

export const fromPixiToPose2D = (point: RotablePoint): Pose2D => {
    if (point === undefined) {
        console.error("ERROR")
    }
    const { x: pixiX, y: pixiY, theta } = point;
    //To transform the coordinates we proportionate the coordinates
    const x = (pixiX * MAX_SIDE_ROS) / MAX_SIDE_WEB
    let y = (pixiY * MAX_SIDE_ROS) / MAX_SIDE_WEB
    //as both maps have different y axis orientations we inverse the value of them 
    y = Math.abs(MAX_SIDE_ROS - y)
    return ({
        x, y, theta
    })
}