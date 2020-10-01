import ROSLIB from "roslib";
import { ROS_PORT } from "../constants";
type ROSConfig = {
    url: string
}
export const rosconfig = {
    url: `ws://0.0.0.0:${ROS_PORT}`,
}

export const initROS = (callback: (status: boolean) => void): ROSLIB.Ros => {
    const ros = new ROSLIB.Ros(rosconfig)
    ros.on('connection', function () {
        console.log('Connected to websocket server.');
        callback(true)
    });

    ros.on('error', function (error) {
        console.log('Error connecting to websocket server: ', error);
        callback(false)

    });

    ros.on('close', function () {
        console.log('Connection to websocket server closed.');
        callback(false)

    });
    return ros;
}