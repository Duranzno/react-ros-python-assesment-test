import React from 'react';
import {
    Point,
} from 'pixi.js';

import {
    // Container,
    Graphics,
} from "@inlet/react-pixi";
import { PIXI_CENTER_WEB_POINT } from '../../constants';
export type TrailPointsProps = {
    points: Point[];
    showPoints?: boolean
};


export const TrailGraphic: React.FC<TrailPointsProps> = ({ points, showPoints }) => {
    return (

        <Graphics
            position={{ x: 0, y: 0 }}
            // scale={0.01}
            draw={g => {
                g.clear();

                g.lineStyle(2, 0xFFBC5E);
                g.moveTo(points[0].x, points[0].y);

                for (let i = 1; i < points.length; i++) {
                    g.lineTo(points[i].x, points[i].y);
                }

                if (showPoints) {
                    for (let i = 1; i < points.length; i++) {
                        g.beginFill(0xFFFD2B);
                        g.drawCircle(points[i].x, points[i].y, 1);
                        g.endFill();
                    }
                }
            }} />
    )
}
TrailGraphic.defaultProps = {
    showPoints: false
}

let points: Point[] = [PIXI_CENTER_WEB_POINT]
const setPoints = (v: Point[]) => points = v
interface TrailProps {
    point: Point,
    showPoints?: boolean
}
export const TrailVisualization: React.FC<TrailProps> = ({ point, showPoints }) => {
    React.useEffect((
    ) => {
        const np = [...points, point];
        const emptyTrail = points.length === 0
        const currentLast = points[points.length - 1]
        const isNewPoint = points.length > 0
            && (currentLast.x !== point.x || currentLast.y !== point.y)
        if ((emptyTrail || isNewPoint)) {
            setPoints(np)
            // console.log(JSON.stringify(points))
        }

    }, [point]);
    return <TrailGraphic points={points} showPoints={showPoints} />
}
TrailVisualization.defaultProps = {
    showPoints: false
}
