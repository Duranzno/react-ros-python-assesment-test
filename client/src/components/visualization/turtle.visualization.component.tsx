import { Sprite } from '@inlet/react-pixi'
import React from 'react'
import { RotablePoint } from '../../models'

interface Props {
    point: RotablePoint
}

export const TurtleVisualization: React.FC<Props> = ({ point: turtleposition }) => {
    return (
        <Sprite
            image={`${process.env.PUBLIC_URL}/images/turtles/turtle.png`}
            anchor={0.5}
            position={{ ...turtleposition }}
            rotation={-1 * turtleposition.theta}
        />

    )
}
