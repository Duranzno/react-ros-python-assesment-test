import React, { useCallback, useEffect } from 'react'
import ROSLIB from 'roslib'

import { fromTurtlesimToPixi, Pose2D, GoActionFeedback, GoActionResult } from '../models';
import { PIXI_CENTER_WEB } from '../constants';
import { MapVisualization } from './visualization';
import { Point } from 'pixi.js';


interface MapControllerProps {
    ros: ROSLIB.Ros;
    className: string;
    isConnected: boolean,
}

export const MapController: React.FC<MapControllerProps> = ({ ros, className, isConnected }) => {
    const [cur, setCur] = React.useState(PIXI_CENTER_WEB)
    const [trail, setTrail] = React.useState<Point[]>([])
    const poseTopic = new ROSLIB.Topic({
        ros, name: "turtle1/pose", messageType: "turtlesim/Pose", "throttle_rate": 100
    })


    const subscription = useCallback((message: ROSLIB.Message): void => {
        const pose = message as Pose2D;
        if (cur !== undefined) {
            const newPoint = (fromTurtlesimToPixi(pose));
            if (cur.x === newPoint.x && cur.y === newPoint.y) {
                // console.log("They are equal no need to move")
            }
            else {
                // console.log(motion)
                // console.log(`Turtle willmove from (${currentPoint.x},${currentPoint.y}) to (${newPoint.x},${newPoint.y}) `)
                // updatePoint(newPoint)
                setCur(newPoint);
                // console.log(`Turtlesim (${pose.x},${pose.y},${pose.theta})| Pixi(${newPoint.x},${newPoint.y},${newPoint.theta}) `);
            }

        } else {
            console.log("Current Point is not defined");
            // console.log(currentPoint)
        }

    }, [cur]);

    useEffect(() => {
        if (isConnected && cur.x === PIXI_CENTER_WEB.x && cur.y === PIXI_CENTER_WEB.y) {
            poseTopic.subscribe(subscription)
        }
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [isConnected, cur])
    // // Actionlib Client 

    const handleNewGoal = (goalMessage: Pose2D): void => {

        const actionClient = new ROSLIB.ActionClient({
            ros: ros,
            actionName: 'backend/GoAction',
            serverName: "go_to_goal",
            timeout: 50000,
        })
        // console.log(goalMessage)
        const g = new ROSLIB.Goal({
            actionClient,
            goalMessage
        });
        g.on('feedback', (feedback: GoActionFeedback | any) => {
            console.log('Feedback: ' + JSON.stringify(feedback));
        });
        g.on('result', (event: GoActionResult | any) => {
            console.log('Final Result: ' + JSON.stringify(event));
        });
        g.send()
        const newCur = fromTurtlesimToPixi(goalMessage)
        setCur(newCur)
        // console.log(cur)
    }

    return (<MapVisualization onNewGoal={handleNewGoal} turtleposition={cur} className={className} />)
}