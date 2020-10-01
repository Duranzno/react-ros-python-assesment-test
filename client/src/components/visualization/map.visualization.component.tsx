import React, { useEffect } from 'react'
import { Sprite, Stage } from '@inlet/react-pixi'
import { Point, Texture } from 'pixi.js';

import { Pose2D, fromPixiToPose2D, RotablePoint } from '../../models';
import { MAX_SIDE_WEB } from '../../constants';
import { TrailVisualization } from './trail.visualization.component';
import { TurtleVisualization } from './turtle.visualization.component';

export interface MapVisualizationProps {
    turtleposition: RotablePoint,
    onNewGoal: (goalMessage: Pose2D) => void,
    className?: string
}
export const MapVisualization: React.FC<MapVisualizationProps> = ({ turtleposition, onNewGoal, className }) => {

    const goToGoal = (goalInWeb: Point) => {
        const goalMessage = fromPixiToPose2D({ ...goalInWeb, theta: 0 })
        onNewGoal(goalMessage)
        // console.log(currentPoint)
    }
    // const [i, setI] = useState(0);

    useEffect(() => {
        let raf: number;

        const loop = () => {
            // raf = requestAnimationFrame(loop);
            // console.log(`New animation frame raf:${raf}, i:${i}`)
            // setI(i => i + 0.1);
        };

        //  raf= 
        requestAnimationFrame(loop);

        return () => {
            cancelAnimationFrame(raf);
        }
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [turtleposition]);

    return (
        <Stage
            raf={false}
            className={className}
            renderOnComponentChange={true}
            width={MAX_SIDE_WEB} height={MAX_SIDE_WEB} options={{ transparent: true }}>
            <Sprite width={MAX_SIDE_WEB} height={MAX_SIDE_WEB}
                interactive={true}
                pointerdown={({ data: { global: goalInWeb } }): void =>
                    goToGoal(goalInWeb)
                }
                texture={Texture.WHITE}
                tint={0x4556FF} />
            <TurtleVisualization point={turtleposition} />
            <TrailVisualization point={new Point(turtleposition.x, turtleposition.y)} />
        </Stage >
    )
}
MapVisualization.defaultProps = {
    className: '',
}